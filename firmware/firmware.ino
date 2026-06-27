// Atom-Matrix 震动传感器：MPU6886 100Hz 采样 -> Detector -> 事件中整屏红灯快闪
// 事件结束把摘要 HTTPS POST 给家里的 iot 服务；断网 NVS 缓冲, 重连补发。
//
// 任务划分：
//   采样任务(core 0)  只读 IMU + 喂检测器 + 驱动 LED + 把结束的事件入队 + 发布遥测, 绝不被网络阻塞。
//   loop(core 1)      WiFi 维护 / NTP / 取队列 POST 事件 / 拉远程配置 / 心跳(带遥测) / 补发缓冲。
// 贴电视后只靠网络调参: 设备每 15s 拉一次, 热更新检测门槛, 并存 NVS。
#include <M5Atom.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>
#include "detector.h"
#include "jma_intensity.h"
#include "config.h"

static const char*    FW_VER = "eq-1.1.1";
static const double   FS = 100.0;
static const uint32_t WIFI_TIMEOUT_MS = 10000;
static const int      NVS_RING = 16;          // 断网最多缓冲的事件数

Detector detector(FS, DetConfig{});            // 默认门槛(floor30/K6/...), 启动后由 NVS/服务器覆盖
static char g_mac[18] = {0};
QueueHandle_t g_evQ = nullptr;                 // EventSummary 队列(采样任务->loop)
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
// 拉服务器配置; 与上次不同才热更新到采样任务 + 存 NVS。服务器是配置权威源。
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

static void buildEventBody(const EventSummary& ev, time_t evEpoch, char* body, size_t n){
  long end_ts   = evEpoch ? (long)evEpoch : 0;
  long start_ts = evEpoch ? (long)(evEpoch - (long)(ev.dur_s+0.5)) : 0;
  int p = snprintf(body, n,
    "{\"mac\":\"%s\",\"fw\":\"%s\",\"start_ts\":%ld,\"end_ts\":%ld,"
    "\"dur_s\":%.1f,\"peak_gal\":%.1f,\"peak_h_gal\":%.1f,\"peak_v_gal\":%.1f,"
    "\"rms_gal\":%.1f,\"shindo\":%.1f,\"shindo_scale\":\"%s\",\"rssi\":%d,\"clipped\":%d,"
    "\"env_dt_ms\":100,\"env_tot\":[",
    g_mac, FW_VER, start_ts, end_ts, ev.dur_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
    ev.rms_gal, ev.keisoku, shindo_scale(ev.keisoku), WiFi.RSSI(), ev.clipped?1:0);
  // 包络三数组(整数 gal), 留 16 字节安全余量防越界
  auto appendArr = [&](const float* arr){
    for(int i=0; i<ev.env_n && p < (int)n-16; i++)
      p += snprintf(body+p, n-p, i? ",%d":"%d", (int)lroundf(arr[i]));
  };
  appendArr(ev.env_tot);
  p += snprintf(body+p, n-p, "],\"env_h\":[");   appendArr(ev.env_h);
  p += snprintf(body+p, n-p, "],\"env_v\":[");   appendArr(ev.env_v);
  p += snprintf(body+p, n-p, "]}");
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
    // LED 在采样核(core0)驱动, 永不被联网阻塞: 持续晃动(≥led_min)才整屏红快闪, 否则全灭。
    if(detector.led_on()){
      if(millis()-lastBlink>50){ lastBlink=millis(); ledState=!ledState;
        M5.dis.fillpix(ledState?CRGB(0xff0000):CRGB(0x000000)); }
    } else if(ledState){ ledState=false; M5.dis.clear(); }
    EventSummary ev;
    if(detector.poll_event(ev)){
      Serial.printf("[event] dur=%.1fs peak=%.1fgal H=%.1f V=%.1f shindo=%.1f(%s) clip=%d\n",
                    ev.dur_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
                    ev.keisoku, shindo_scale(ev.keisoku), ev.clipped);
      if(g_evQ) xQueueSend(g_evQ, &ev, 0);       // 交给 loop 去 POST(采样不阻塞)
    }
    vTaskDelayUntil(&last, period);
  }
}

void setup(){
  M5.begin(true, true, true);     // Serial, I2C(MPU6886), Display(5x5)
  M5.IMU.Init();
  M5.IMU.SetAccelFsr(MPU6886::AFS_4G);   // ±4g, 与 detector 触顶判定一致
  M5.dis.clear();                 // 空闲全灭
  Serial.begin(115200);
  // 启动用 NVS 缓存的配置(若有), 否则保持默认; 服务器联通后会再覆盖。
  g_appliedCfg = detector.get_config();
  if(loadConfigNVS(g_appliedCfg)){
    detector.set_config(g_appliedCfg);
    Serial.printf("[cfg] 从 NVS 载入 floor=%.1f K=%.1f minK=%.2f\n",
                  g_appliedCfg.floor_min, g_appliedCfg.k, g_appliedCfg.min_keisoku);
  }
  g_haveApplied = true;
  g_evQ = xQueueCreate(4, sizeof(EventSummary));   // EventSummary 含包络较大, 队列短即可
  wifiConnect();
  if(WiFi.status()==WL_CONNECTED) fetchConfig();    // 开机即拉一次服务器配置
  xTaskCreatePinnedToCore(sampleTask, "sample", 12288, nullptr, 2, nullptr, 0);
}

void loop(){
  // LED 已移到采样任务(core0)驱动, loop 只管联网。
  // 1) 取队列里的事件 POST(失败转 NVS 缓冲)
  EventSummary ev;
  if(g_evQ && xQueueReceive(g_evQ, &ev, 0)==pdTRUE){
    static char body[4096];   // 含包络, 用 static 避免占 loop 栈
    buildEventBody(ev, nowEpoch(), body, sizeof(body));
    if(!postJson("", body)) bufferEvent(body);
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
      postJson("/heartbeat", hb);
    }
  }
  delay(5);
}
