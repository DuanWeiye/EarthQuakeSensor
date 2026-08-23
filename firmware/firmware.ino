// Atom-Matrix 震动传感器：MPU6886 100Hz 采样 -> Detector -> 事件中整屏红灯快闪
// 事件结束把摘要 HTTPS POST 给家里的 iot 服务；断网 NVS 缓冲, 重连补发。
//
// 任务划分：
//   采样任务(core 0)  只读 IMU + 喂检测器 + 驱动 LED + 把结束的事件入队 + 发布遥测, 绝不被网络阻塞。
//   loop(core 1)      WiFi 维护 / NTP / 取队列 POST 事件 / 拉远程配置 / 心跳(带遥测) / 补发缓冲。
// 贴电视后只靠网络调参: 设备每 15s 拉一次, 热更新检测门槛, 并存 NVS。
#include <M5Atom.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include "esp_ota_ops.h"
#include "detector.h"
#include "jma_intensity.h"
#include "config.h"

static const char*    FW_VER = "eq-1.3.0";    // + 加速度计 DLPF 抗混叠(21Hz) + 事件时间戳按采样时刻精确推算
static const double   FS = 100.0;
static const uint32_t WIFI_TIMEOUT_MS = 10000;
static const int      NVS_RING = 16;          // 断网最多缓冲的事件数(离线只存标量, 省 NVS)

// loopTask 栈加大到 16KB: WiFiClientSecure 的 TLS 握手 + httpUpdate OTA 远超默认 8KB,
// 否则 loop 里 fetchConfig/心跳/OTA 的 TLS 会触发栈溢出(stack canary)崩溃。
SET_LOOP_TASK_STACK_SIZE(16384);

Detector detector(FS, DetConfig{});            // 默认门槛(floor30/K6/...), 启动后由 NVS/服务器覆盖
static char g_mac[18] = {0};
// 采样任务 -> loop 的队列项: 摘要 + 事件起止的设备 millis 时刻。
// 起止时刻由采样计数精确推算(不再用"收尾时刻-时长"反推), 消除 hold-off/队列延迟带来的固定偏晚。
struct QueuedEvent { EventSummary ev; uint32_t startMs, endMs; };
QueueHandle_t g_evQ = nullptr;                 // QueuedEvent 队列(采样任务->loop)
Preferences prefs;

// --- 遥测(core0 发布, loop 读): 用 volatile float, 32位读写无撕裂; 跨核竞争仅丢极少量, 无害 ---
volatile float g_noiseGal = 0;                 // 当前噪声底
volatile float g_peakGal  = 0;                 // 自上次心跳以来 |动态| 峰值(心跳时取走清零)

// --- 配置热更新握手: loop 写 g_pendingCfg 后置位 g_cfgPending, 采样任务在安全点取用 ---
portMUX_TYPE g_cfgMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_cfgPending = false;
DetConfig g_pendingCfg;
DetConfig g_appliedCfg;                         // 最近一次已下发的配置(比较, 变了才下发/存盘)
bool g_haveApplied = false;

// ---------- IMU ----------
// MPU6886 加速度计数字低通(ACCEL_CONFIG2.A_DLPF_CFG): 0=218Hz 1=218Hz 2=99Hz 3=44.8Hz 4=21.2Hz 5=10.2Hz 6=5.1Hz。
// M5Atom 库 Init() 不碰该寄存器(默认 218Hz 带宽, 内部 1kHz 采样), 我们只以 100Hz 读取, 50Hz 以上的
// 传感器噪声/高频机械振动会混叠进 0~50Hz 带内, 抬高噪声底(实测 ~2.3gal)。地震能量 <10Hz, 取 21.2Hz
// 既抗混叠又不伤 JMA 通带(G(f) 在 10Hz 本就只剩 0.22)。库把 I2C 写封成 private, 故用同一条 Wire1 直写。
static void imuSetAccelDlpf(uint8_t cfg){
  const uint8_t ADDR = 0x68, ACCEL_CONFIG2 = 0x1D;
  Wire1.beginTransmission(ADDR); Wire1.write(ACCEL_CONFIG2); Wire1.write(cfg & 0x07); Wire1.endTransmission();
  Wire1.beginTransmission(ADDR); Wire1.write(ACCEL_CONFIG2); Wire1.endTransmission(false);
  uint8_t rb = 0xFF; if(Wire1.requestFrom(ADDR, (uint8_t)1) == 1) rb = Wire1.read();
  Serial.printf("[imu] ACCEL_CONFIG2=0x%02X (%s)\n", rb, (rb & 0x07)==(cfg & 0x07) ? "DLPF ok" : "DLPF 写入失败");
}

// ---------- 时间 ----------
static time_t nowEpoch(){ time_t t=time(nullptr); return (t>1700000000)?t:0; } // 未同步返回0

// ---------- WiFi ----------
static bool wifiConnect(){
  if(WiFi.status()==WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<WIFI_TIMEOUT_MS){ delay(200); }
  if(WiFi.status()==WL_CONNECTED){
    strncpy(g_mac, WiFi.macAddress().c_str(), sizeof(g_mac)-1);
    configTime(9*3600, 0, "ntp.nict.jp", "pool.ntp.org");   // JST(UTC+9)
    Serial.printf("[wifi] connected ip=%s mac=%s rssi=%d\n",
                  WiFi.localIP().toString().c_str(), g_mac, WiFi.RSSI());
    return true;
  }
  Serial.println("[wifi] connect failed");
  return false;
}

// ---------- HTTP ----------
// path 为相对 REPORT_BASE 的后缀: ""=事件上报, "/config"=拉配置, "/heartbeat"=心跳。
static bool postJson(const char* path, const char* body){
  if(WiFi.status()!=WL_CONNECTED) return false;
  WiFiClientSecure cli; cli.setInsecure(); cli.setTimeout(8);
  HTTPClient http; String url=String(REPORT_BASE)+path;
  if(!http.begin(cli, url)) return false;
  http.addHeader("Content-Type","application/json");
  int code=http.POST((uint8_t*)body, strlen(body));
  http.end();
  Serial.printf("[post] %s HTTP %d\n", path, code);
  return code>0 && code<400;
}

static String getJson(const char* path){
  if(WiFi.status()!=WL_CONNECTED) return String();
  WiFiClientSecure cli; cli.setInsecure(); cli.setTimeout(8);
  HTTPClient http; String url=String(REPORT_BASE)+path;
  if(!http.begin(cli, url)) return String();
  int code=http.GET();
  String body = (code==200) ? http.getString() : String();
  http.end();
  return body;
}

// ---------- 远程配置(NVS 缓存 + 服务器拉取) ----------
static void saveConfigNVS(const DetConfig& c){
  prefs.begin("eqcfg", false);
  prefs.putDouble("k",c.k);          prefs.putDouble("fm",c.floor_min);
  prefs.putDouble("ho",c.holdoff_s); prefs.putDouble("md",c.min_dur_s);
  prefs.putDouble("led",c.led_min_s);prefs.putDouble("mk",c.min_keisoku);
  prefs.end();
}
static bool loadConfigNVS(DetConfig& c){
  prefs.begin("eqcfg", true);
  double k = prefs.getDouble("k", NAN);
  bool has = !isnan(k);
  if(has){
    c.k=k;                          c.floor_min =prefs.getDouble("fm", c.floor_min);
    c.holdoff_s=prefs.getDouble("ho",c.holdoff_s); c.min_dur_s=prefs.getDouble("md",c.min_dur_s);
    c.led_min_s=prefs.getDouble("led",c.led_min_s);c.min_keisoku=prefs.getDouble("mk",c.min_keisoku);
  }
  prefs.end();
  return has;
}
static bool cfgEqual(const DetConfig& a, const DetConfig& b){
  auto eq=[](double x,double y){ return fabs(x-y)<1e-6; };
  return eq(a.k,b.k)&&eq(a.floor_min,b.floor_min)&&eq(a.holdoff_s,b.holdoff_s)
       &&eq(a.min_dur_s,b.min_dur_s)&&eq(a.led_min_s,b.led_min_s)&&eq(a.min_keisoku,b.min_keisoku);
}
// ---------- OTA(长按按钮触发: 拉 /config 比对版本 -> HTTPS 下载 .bin 并重启; core 自带回滚) ----------
// LED 状态(core0 渲染, 按钮/core1 设置): 检查=蓝常亮, 升级=蓝闪; OTA 优先于事件红闪。
enum { LED_NORMAL = 0, LED_OTA_CHECK, LED_OTA_UPGRADE };
static volatile int  g_ledMode = LED_NORMAL;
static volatile bool g_otaRequest = false;     // core0 长按置位 -> core1 执行 OTA
static bool     g_otaPending = false;          // 本镜像刚 OTA、处于"待确认"态
static uint32_t g_bootMs = 0;

// 开机检查: 若是待确认镜像 -> 联网成功前不提交, 崩溃/超时自动回滚到旧固件。
static void otaCheckBoot(){
  const esp_partition_t* run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if(run && esp_ota_get_state_partition(run, &st)==ESP_OK && st==ESP_OTA_IMG_PENDING_VERIFY){
    g_otaPending = true;
    Serial.println("[ota] 新固件待确认: 联网成功才提交, 否则回滚旧版");
  }
}
// 心跳成功后调用: 确认新固件有效(取消回滚)。
static void otaConfirmIfPending(){
  if(g_otaPending){
    esp_ota_mark_app_valid_cancel_rollback();
    g_otaPending = false;
    Serial.println("[ota] 新固件已确认有效");
  }
}
// 版本不同则从 REPORT_BASE+"/firmware" 下载升级; 成功自动重启进新固件, 失败继续跑旧固件。
// URL 由 config.h 的 REPORT_BASE 自拼, 服务器地址只此一处(不进 /config 响应)。
static void otaMaybeUpdate(const char* ver){
  if(!ver || !*ver || strcmp(ver, FW_VER)==0) return;          // 无广播 / 已是最新
  String url = String(REPORT_BASE) + "/firmware";
  Serial.printf("[ota] 新版本 %s (当前 %s), 升级: %s\n", ver, FW_VER, url.c_str());
  WiFiClientSecure cli; cli.setInsecure(); cli.setTimeout(20);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return r = httpUpdate.update(cli, url);
  if(r==HTTP_UPDATE_FAILED)
    Serial.printf("[ota] 失败 %d: %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
}

// 长按按钮触发(core1/loop 调用): 蓝灯常亮=拉 /config 比对版本, 蓝灯闪=下载升级; 成功重启进新固件。
static void handleOtaButton(){
  if(!g_otaRequest) return;
  g_otaRequest = false;
  if(WiFi.status()!=WL_CONNECTED){ Serial.println("[ota] 未联网, 取消按钮升级"); return; }
  g_ledMode = LED_OTA_CHECK;                       // 蓝灯常亮: 检查中
  Serial.println("[ota] 按钮触发: 检查新版本...");
  String ver, body = getJson("/config");
  if(body.length()){
    JsonDocument doc;
    if(!deserializeJson(doc, body)){
      JsonObject ota = doc["ota"];
      if(!ota.isNull()) ver = (const char*)(ota["version"] | "");
    }
  }
  if(ver.length() && ver != FW_VER){
    g_ledMode = LED_OTA_UPGRADE;                    // 蓝灯闪: 升级中
    otaMaybeUpdate(ver.c_str());                    // 成功重启不返回; 失败落到下面
  } else {
    Serial.printf("[ota] 无新版本(广播=%s, 当前=%s)\n", ver.length()?ver.c_str():"无", FW_VER);
  }
  g_ledMode = LED_NORMAL;                           // 无更新/失败: 回正常
}

// 拉服务器配置; 与上次不同才热更新到采样任务 + 存 NVS。服务器是配置权威源。
// (OTA 不在此自动触发, 改由长按按钮 handleOtaButton 主动检查升级。)
static void fetchConfig(){
  String body = getJson("/config");
  if(!body.length()) return;
  JsonDocument doc;
  if(deserializeJson(doc, body)) return;
  JsonObject c = doc["config"];
  if(c.isNull()) return;
  DetConfig n = g_appliedCfg;                    // 缺字段时沿用当前值
  n.k         = c["k"]          | n.k;
  n.floor_min = c["floor_min"]  | n.floor_min;
  n.holdoff_s = c["holdoff_s"]  | n.holdoff_s;
  n.min_dur_s = c["min_dur_s"]  | n.min_dur_s;
  n.led_min_s = c["led_min_s"]  | n.led_min_s;
  n.min_keisoku = c["min_keisoku"] | n.min_keisoku;
  if(g_haveApplied && cfgEqual(n, g_appliedCfg)) return;
  portENTER_CRITICAL(&g_cfgMux);
  g_pendingCfg = n; g_cfgPending = true;
  portEXIT_CRITICAL(&g_cfgMux);
  g_appliedCfg = n; g_haveApplied = true;
  saveConfigNVS(n);
  Serial.printf("[cfg] 下发 K=%.1f floor=%.1f hold=%.1f minDur=%.1f led=%.1f minK=%.2f\n",
                n.k, n.floor_min, n.holdoff_s, n.min_dur_s, n.led_min_s, n.min_keisoku);
}

// includeEnv=true: 含 100ms 自适应包络(在线上报); false: 只存标量(离线缓冲省 NVS)。
// start_ts/end_ts: 事件起止 epoch(未对时则 0, 服务器回落为收到时刻)。
static void buildEventBody(const EventSummary& ev, long start_ts, long end_ts, char* body, size_t n, bool includeEnv){
  int p = snprintf(body, n,
    "{\"mac\":\"%s\",\"fw\":\"%s\",\"start_ts\":%ld,\"end_ts\":%ld,"
    "\"dur_s\":%.1f,\"peak_gal\":%.1f,\"peak_h_gal\":%.1f,\"peak_v_gal\":%.1f,"
    "\"rms_gal\":%.1f,\"shindo\":%.1f,\"shindo_scale\":\"%s\",\"rssi\":%d,\"clipped\":%d",
    g_mac, FW_VER, start_ts, end_ts, ev.dur_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
    ev.rms_gal, ev.keisoku, shindo_scale(ev.keisoku), WiFi.RSSI(), ev.clipped?1:0);
  if(includeEnv){
    // env_dt_ms 自适应(长事件会 >100); 包络三数组(整数 gal), 留 16 字节安全余量防越界
    p += snprintf(body+p, n-p, ",\"env_dt_ms\":%d,\"env_tot\":[", (int)(ev.env_dt_ms+0.5));
    auto appendArr = [&](const float* arr){
      for(int i=0; i<ev.env_n && p < (int)n-16; i++)
        p += snprintf(body+p, n-p, i? ",%d":"%d", (int)lroundf(arr[i]));
    };
    appendArr(ev.env_tot);
    p += snprintf(body+p, n-p, "],\"env_h\":[");   appendArr(ev.env_h);
    p += snprintf(body+p, n-p, "],\"env_v\":[");   appendArr(ev.env_v);
    p += snprintf(body+p, n-p, "]}");
  } else {
    p += snprintf(body+p, n-p, "}");
  }
}

// ---------- NVS 环形缓冲(断网补发) ----------
static void bufferEvent(const char* body){
  prefs.begin("eqbuf", false);
  int tail=prefs.getInt("tail",0);
  char key[8]; snprintf(key,sizeof(key),"e%d", tail%NVS_RING);
  prefs.putString(key, body);
  prefs.putInt("tail", tail+1);
  prefs.end();
  Serial.println("[buf] event 缓冲(待补发)");
}
static void flushBuffer(){
  if(WiFi.status()!=WL_CONNECTED) return;
  prefs.begin("eqbuf", false);
  int head=prefs.getInt("head",0), tail=prefs.getInt("tail",0);
  while(head<tail){
    char key[8]; snprintf(key,sizeof(key),"e%d", head%NVS_RING);
    String b=prefs.getString(key,"");
    if(b.length() && !postJson("", b.c_str())) break;  // 仍失败下次再补
    head++; prefs.putInt("head", head);
  }
  prefs.end();
}

// ---------- 采样任务(core 0) ----------
void sampleTask(void*){
  const TickType_t period = pdMS_TO_TICKS(10);   // 100Hz
  TickType_t last = xTaskGetTickCount();
  float ax, ay, az;
  bool ledState=false; uint32_t lastBlink=0;
  uint32_t btnDownMs=0; bool btnLongFired=false;   // 顶部按钮长按检测
  for(;;){
    // 安全点应用 loop 下发的新配置(临界区只复制结构体, set_config 在本核执行)
    if(g_cfgPending){
      portENTER_CRITICAL(&g_cfgMux);
      DetConfig c = g_pendingCfg; g_cfgPending = false;
      portEXIT_CRITICAL(&g_cfgMux);
      detector.set_config(c);
    }
    M5.IMU.getAccelData(&ax, &ay, &az);          // 单位 g
    detector.push(ax*980.665f, ay*980.665f, az*980.665f);
    g_noiseGal = (float)detector.noise_floor();
    float pk = detector.take_recent_peak();
    if(pk > g_peakGal) g_peakGal = pk;
    // 顶部按钮: 长按 3s 触发 OTA(置跨核标志, 由 loop/core1 执行)
    M5.update();
    if(M5.Btn.isPressed()){
      if(btnDownMs==0) btnDownMs=millis();
      if(!btnLongFired && millis()-btnDownMs>=3000){ btnLongFired=true; g_otaRequest=true; }
    } else { btnDownMs=0; btnLongFired=false; }
    // LED(core0 单一驱动): OTA 蓝灯优先于事件红闪。检查=蓝常亮, 升级=蓝闪, 事件=红快闪, 否则灭。
    int lm = g_ledMode;
    if(lm==LED_OTA_CHECK){
      M5.dis.fillpix(CRGB(0x0000ff)); ledState=true;                   // 检查: 蓝常亮
    } else if(lm==LED_OTA_UPGRADE){
      if(millis()-lastBlink>200){ lastBlink=millis(); ledState=!ledState;
        M5.dis.fillpix(ledState?CRGB(0x0000ff):CRGB(0x000000)); }      // 升级: 蓝闪
    } else if(detector.led_on()){
      if(millis()-lastBlink>50){ lastBlink=millis(); ledState=!ledState;
        M5.dis.fillpix(ledState?CRGB(0xff0000):CRGB(0x000000)); }      // 事件: 红快闪
    } else if(ledState){ ledState=false; M5.dis.clear(); }
    QueuedEvent q;
    if(detector.poll_event(q.ev)){
      const EventSummary& ev = q.ev;
      // 起止时刻: 用"当前样本时刻 - 事件样本时刻"折算成设备 millis(采样严格 100Hz, 误差 <1 tick)
      uint32_t nowMs = millis(); double tnow = detector.now_s();
      q.startMs = nowMs - (uint32_t)((tnow - ev.start_s)*1000.0 + 0.5);
      q.endMs   = nowMs - (uint32_t)((tnow - ev.end_s)*1000.0 + 0.5);
      Serial.printf("[event] dur=%.1fs peak=%.1fgal H=%.1f V=%.1f shindo=%.1f(%s) clip=%d\n",
                    ev.dur_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
                    ev.keisoku, shindo_scale(ev.keisoku), ev.clipped);
      if(g_evQ) xQueueSend(g_evQ, &q, 0);        // 交给 loop 去 POST(采样不阻塞)
    }
    vTaskDelayUntil(&last, period);
  }
}

void setup(){
  g_bootMs = millis();
  M5.begin(true, true, true);     // Serial, I2C(MPU6886), Display(5x5)
  otaCheckBoot();                 // OTA: 成功则清目标; 待确认镜像标记待提交(否则回滚)
  M5.IMU.Init();
  M5.IMU.SetAccelFsr(MPU6886::AFS_4G);   // ±4g, 与 detector 触顶判定一致
  imuSetAccelDlpf(4);                    // 抗混叠: 加速度计 DLPF 21.2Hz(库默认 218Hz, 100Hz 采样会把高频噪声折进带内)
  M5.dis.clear();                 // 空闲全灭
  Serial.begin(115200);
  detector.begin();               // 堆就绪后预构建流式震度 FIR 抽头(避免全局构造期/首事件分配)
  // 启动用 NVS 缓存的配置(若有), 否则保持默认; 服务器联通后会再覆盖。
  g_appliedCfg = detector.get_config();
  if(loadConfigNVS(g_appliedCfg)){
    detector.set_config(g_appliedCfg);
    Serial.printf("[cfg] 从 NVS 载入 floor=%.1f K=%.1f minK=%.2f\n",
                  g_appliedCfg.floor_min, g_appliedCfg.k, g_appliedCfg.min_keisoku);
  }
  g_haveApplied = true;
  g_evQ = xQueueCreate(4, sizeof(QueuedEvent));    // 含包络较大(~3.7KB/项), 队列短即可
  wifiConnect();
  if(WiFi.status()==WL_CONNECTED) fetchConfig();    // 开机即拉一次服务器配置
  // 正常启动标识: 绿灯闪一下(在采样任务启动前驱动, 无 LED 竞争)。OTA 后新固件启动也走这里。
  M5.dis.fillpix(CRGB(0x00ff00)); delay(150); M5.dis.clear();
  xTaskCreatePinnedToCore(sampleTask, "sample", 12288, nullptr, 2, nullptr, 0);
}

void loop(){
  handleOtaButton();   // 长按顶部按钮触发的 OTA(检查+升级); LED 已由 core0 驱动
  // 1) 取队列里的事件 POST(失败转 NVS 缓冲)
  static QueuedEvent q;       // ~3.7KB, static 避免占 loop 栈
  if(g_evQ && xQueueReceive(g_evQ, &q, 0)==pdTRUE){
    static char body[8192];   // 含 300 点自适应包络, static 避免占 loop 栈
    // 事件起止 epoch = 当前 epoch - (当前 millis - 事件 millis): 不受 hold-off 与队列/网络延迟影响
    time_t now = nowEpoch(); uint32_t ms = millis();
    long start_ts = now ? (long)now - (long)((ms - q.startMs + 500) / 1000) : 0;
    long end_ts   = now ? (long)now - (long)((ms - q.endMs   + 500) / 1000) : 0;
    buildEventBody(q.ev, start_ts, end_ts, body, sizeof(body), true);        // 完整(含包络)上报
    if(!postJson("", body)){
      static char cbody[1024];
      buildEventBody(q.ev, start_ts, end_ts, cbody, sizeof(cbody), false);   // 离线只存标量(无包络), 省 NVS
      bufferEvent(cbody);
    }
  }

  // 2) WiFi 维护; 每 15s: 拉远程配置 + 补发 + 心跳(带噪声/峰值遥测)
  static uint32_t lastWifi=0, lastTick=0;
  if(WiFi.status()!=WL_CONNECTED && millis()-lastWifi>5000){ lastWifi=millis(); wifiConnect(); }
  if(millis()-lastTick>15000){
    lastTick=millis();
    if(WiFi.status()==WL_CONNECTED){
      fetchConfig();                              // 拉配置热更新
      flushBuffer();
      float peak = g_peakGal; g_peakGal = 0;      // 取走峰值(本窗口最大)
      const DetConfig& a = g_appliedCfg;          // 回报设备实际生效的配置(供面板确认下发到位)
      char hb[360]; snprintf(hb,sizeof(hb),
        "{\"mac\":\"%s\",\"fw\":\"%s\",\"rssi\":%d,\"uptime_ms\":%lu,"
        "\"noise_gal\":%.1f,\"peak_gal\":%.1f,\"cfg\":{"
        "\"k\":%.1f,\"floor_min\":%.1f,\"holdoff_s\":%.1f,"
        "\"min_dur_s\":%.1f,\"led_min_s\":%.1f,\"min_keisoku\":%.2f}}",
        g_mac, FW_VER, WiFi.RSSI(), (unsigned long)millis(), g_noiseGal, peak,
        a.k, a.floor_min, a.holdoff_s, a.min_dur_s, a.led_min_s, a.min_keisoku);
      if(postJson("/heartbeat", hb)) otaConfirmIfPending();   // 联网成功 -> 确认新固件
    }
    // OTA 待确认超时(5min 仍未联网成功): 重启触发回滚到旧固件
    if(g_otaPending && millis()-g_bootMs > 300000){
      Serial.println("[ota] 待确认超时, 重启回滚"); esp_restart();
    }
  }
  delay(5);
}
