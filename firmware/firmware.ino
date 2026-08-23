// Atom-Matrix 震动传感器：MPU6886 100Hz 采样 -> Detector -> 事件中整屏红灯快闪
// 事件结束把摘要 HTTPS POST 给家里的 iot 服务；断网 NVS 缓冲, 重连补发。
// 事件进行中同时把 100Hz 三轴原始波形(含触发前 WAVE_PRE_S 秒)分块流式上报, 时长不限:
// 块先写进 flash 日志分区(wavelog, 见 partitions.csv), 再由网络核按序上传; 断网时留在 flash, 联网后补传; 断电后开机扫描恢复。
//
// 任务划分：
//   采样任务(core 0)  只读 IMU + 写原始环缓冲 + 喂检测器 + 驱动 LED + 把结束的事件入队 + 发布遥测, 绝不被网络阻塞。
//   loop(core 1)      WiFi 维护 / NTP / 取队列 POST 事件 / 原始波形分块上报 / 拉远程配置 / 心跳(带遥测) / 补发缓冲。
// 检测门槛全部网络热更新: 设备每 15s 拉一次, 并存 NVS。
#include <M5Atom.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include "esp_ota_ops.h"
#include "detector.h"
#include "wavering.h"
#include "wavelog.h"
#include "esp_partition.h"
#include "jma_intensity.h"
#include "config.h"

static const char*    FW_VER = "eq-1.5.0";    // + 原始波形 flash 日志(断网补传/断电留存, 自定义分区表 wavelog 1MB)
static const double   FS = 100.0;
static const uint32_t WIFI_TIMEOUT_MS = 10000;
static const int      NVS_RING = 16;          // 断网最多缓冲的事件数(离线只存标量, 省 NVS)
// 原始波形: 环容量 60s(36KB, setup 里堆分配), 触发前补传 15s(P 波在这里), 每块 5s(3000B 二进制)。
// 环容量 - 预触发 = 可容忍的断传时长(45s); 事件本身不限长。
static const int      WAVE_RING_S  = 60;
static const int      WAVE_PRE_S   = 15;
static const int      WAVE_CHUNK   = 500;
static const int      WAVE_LSB_PER_G = 8192;  // MPU6886 ±4g: 32768/4

// loopTask 栈加大到 16KB: WiFiClientSecure 的 TLS 握手 + httpUpdate OTA 远超默认 8KB,
// 否则 loop 里 fetchConfig/心跳/OTA 的 TLS 会触发栈溢出(stack canary)崩溃。
SET_LOOP_TASK_STACK_SIZE(16384);

Detector detector(FS, DetConfig{});            // 默认门槛(floor30/K6/...), 启动后由 NVS/服务器覆盖
WaveRing g_wave;                               // 原始加速度环(采样核写, loop 读); 缓冲在 setup 里分配
// 原始波形 flash 日志: esp_partition 适配 FlashIO(纯 C++ 的 WaveLog 不认识 IDF)
struct EspFlashIO : FlashIO {
  const esp_partition_t* part = nullptr;
  bool read(uint32_t off, void* buf, size_t len) override { return part && esp_partition_read(part, off, buf, len)==ESP_OK; }
  bool write(uint32_t off, const void* buf, size_t len) override { return part && esp_partition_write(part, off, buf, len)==ESP_OK; }
  bool eraseSector(uint32_t off) override { return part && esp_partition_erase_range(part, off, WaveLog::SECTOR)==ESP_OK; }
};
static EspFlashIO g_wlogIo;
static WaveLog    g_wlog;
static char g_mac[18] = {0};
// 采样任务 -> loop 的队列项: 摘要 + 事件起止的绝对样本序号(与环缓冲/检测器同一计数)。
// 起止时刻由样本序号经时间锚折算成 epoch, 不受 hold-off/队列/网络延迟影响。
struct QueuedEvent { EventSummary ev; uint32_t startIdx, endIdx; };
QueueHandle_t g_evQ = nullptr;                 // QueuedEvent 队列(采样任务->loop)
Preferences prefs;

// --- 进行中事件的状态(core0 发布, loop 读): 供原始波形在事件结束前就开始流式上报 ---
volatile bool     g_evActive   = false;        // 检测器当前有事件进行中
volatile bool     g_evStrongOk = false;        // 强晃跨度已达 min_dur_s(该事件不会被时长门限丢弃)
volatile uint32_t g_evSeq      = 0;            // 事件序号(每次触发 +1)
volatile uint32_t g_evStartIdx = 0;            // 事件起点样本序号
volatile uint32_t g_evEndIdx   = 0;            // 事件结束(含 hold-off)时的样本序号

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
static int64_t nowEpochMs(){
  struct timeval tv; gettimeofday(&tv, nullptr);
  if(tv.tv_sec < 1700000000) return 0;
  return (int64_t)tv.tv_sec*1000 + tv.tv_usec/1000;
}
// 样本序号 -> epoch ms 的"时间锚": 记一次 (序号, 时刻) 对, 之后纯按 10ms/样本折算。
// 同一事件里所有时刻(波形块 t0、事件 start_ms)都经同一锚推算, 彼此严格一致(服务端按 ev 毫秒值关联块与摘要)。
// 锚只在空闲(无事件/无流式上报)时刷新, 吸收采样时钟与 NTP 的缓慢漂移。
static uint32_t g_anchorIdx = 0; static int64_t g_anchorMs = 0;
static void refreshTimeAnchor(){ int64_t now=nowEpochMs(); if(!now) return; g_anchorIdx=g_wave.head(); g_anchorMs=now; }
static int64_t idxToEpochMs(uint32_t idx){
  if(!g_anchorMs) return 0;
  return g_anchorMs + (int64_t)(int32_t)(idx - g_anchorIdx) * (int64_t)(1000.0/FS + 0.5);
}

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
// path 为相对 REPORT_BASE 的后缀(可带查询串): ""=事件上报, "/config"=拉配置, "/heartbeat"=心跳, "/wave?..."=波形块。
static bool postBytes(const char* path, const char* contentType, const uint8_t* data, size_t len){
  if(WiFi.status()!=WL_CONNECTED) return false;
  WiFiClientSecure cli; cli.setInsecure(); cli.setTimeout(8);
  HTTPClient http; String url=String(REPORT_BASE)+path;
  if(!http.begin(cli, url)) return false;
  http.addHeader("Content-Type", contentType);
  int code=http.POST((uint8_t*)data, len);
  http.end();
  Serial.printf("[post] %.*s HTTP %d (%u B)\n", 24, path, code, (unsigned)len);
  return code>0 && code<400;
}
static bool postJson(const char* path, const char* body){
  return postBytes(path, "application/json", (const uint8_t*)body, strlen(body));
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
  prefs.putDouble("ko",c.k_off);     prefs.putDouble("fo",c.floor_off);
  prefs.putDouble("tm",c.tail_max_s);
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
    c.k_off=prefs.getDouble("ko",c.k_off);         c.floor_off=prefs.getDouble("fo",c.floor_off);
    c.tail_max_s=prefs.getDouble("tm",c.tail_max_s);
  }
  prefs.end();
  return has;
}
static bool cfgEqual(const DetConfig& a, const DetConfig& b){
  auto eq=[](double x,double y){ return fabs(x-y)<1e-6; };
  return eq(a.k,b.k)&&eq(a.floor_min,b.floor_min)&&eq(a.holdoff_s,b.holdoff_s)
       &&eq(a.min_dur_s,b.min_dur_s)&&eq(a.led_min_s,b.led_min_s)&&eq(a.min_keisoku,b.min_keisoku)
       &&eq(a.k_off,b.k_off)&&eq(a.floor_off,b.floor_off)&&eq(a.tail_max_s,b.tail_max_s);
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
  n.k_off       = c["k_off"]      | n.k_off;
  n.floor_off   = c["floor_off"]  | n.floor_off;
  n.tail_max_s  = c["tail_max_s"] | n.tail_max_s;
  if(g_haveApplied && cfgEqual(n, g_appliedCfg)) return;
  portENTER_CRITICAL(&g_cfgMux);
  g_pendingCfg = n; g_cfgPending = true;
  portEXIT_CRITICAL(&g_cfgMux);
  g_appliedCfg = n; g_haveApplied = true;
  saveConfigNVS(n);
  Serial.printf("[cfg] 下发 K=%.1f floor=%.1f hold=%.1f minDur=%.1f led=%.1f minK=%.2f Koff=%.1f floorOff=%.1f tailMax=%.0f\n",
                n.k, n.floor_min, n.holdoff_s, n.min_dur_s, n.led_min_s, n.min_keisoku, n.k_off, n.floor_off, n.tail_max_s);
}

// includeEnv=true: 含 100ms 自适应包络(在线上报); false: 只存标量(离线缓冲省 NVS)。
// start_ms/end_ms: 事件起止 epoch 毫秒(未对时则 0, 服务器回落为收到时刻); start_ts/end_ts 为其秒值。
// start_ms 与波形块的 ev 同源同值, 服务端据此把原始波形关联到该事件。
static void buildEventBody(const EventSummary& ev, int64_t start_ms, int64_t end_ms, char* body, size_t n, bool includeEnv){
  int p = snprintf(body, n,
    "{\"mac\":\"%s\",\"fw\":\"%s\",\"start_ts\":%lld,\"end_ts\":%lld,\"start_ms\":%lld,\"end_ms\":%lld,"
    "\"dur_s\":%.1f,\"strong_s\":%.1f,\"peak_gal\":%.1f,\"peak_h_gal\":%.1f,\"peak_v_gal\":%.1f,"
    "\"rms_gal\":%.1f,\"shindo\":%.1f,\"shindo_scale\":\"%s\",\"rssi\":%d,\"clipped\":%d",
    g_mac, FW_VER, (long long)(start_ms/1000), (long long)(end_ms/1000), (long long)start_ms, (long long)end_ms,
    ev.dur_s, ev.strong_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
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

// ---------- 原始波形流式上报(loop/core1) ----------
// 事件强晃跨度一达 min_dur_s(确定不会被时长门限丢弃)即开始: 先补传触发前 WAVE_PRE_S 秒, 之后每满
// WAVE_CHUNK 样本发一块, 事件结束后把余下的发完。块以 (mac, ev=事件起点 epoch ms, seq) 标识, 体为
// 交错 x,y,z int16 原始 ADC(小端), 服务端按 ev 与随后上报的事件摘要(start_ms 同值)关联。
// 事件长度不限; 网络卡住超过"环容量-预触发"则中间丢一段(块 t0 带真实时刻, 服务端能看出缺口)。
// 块的去向: 有 wavelog 分区 -> 先 append 进 flash 日志, 由 waveDrainTick 按记录号顺序上传(断网自然积压,
// 联网后补传; 断电后 begin() 扫描恢复); 无分区(老分区表) -> 退回直传。
struct WaveStream {
  bool active=false, ended=false;
  uint32_t seenSeq=0, evStartIdx=0, next=0, endIdx=0, chunkSeq=0, lastFailMs=0;
  int64_t evMs=0;
};
static WaveStream g_ws;
static int16_t g_waveBuf[WAVE_CHUNK*3];   // 一块 3000B(环读出/日志读出共用, loop 内串行), static 不占 loop 栈

static bool postWaveChunk(int64_t evMs, uint32_t seq, int64_t t0Ms, int n, int fs, int lsbG, const int16_t* data){
  char q[200];
  snprintf(q, sizeof(q), "/wave?mac=%s&ev=%lld&seq=%lu&t0=%lld&fs=%d&n=%d&lsb_g=%d",
           g_mac, (long long)evMs, (unsigned long)seq, (long long)t0Ms, fs, n, lsbG);
  return postBytes(q, "application/octet-stream", (const uint8_t*)data, (size_t)n*3*sizeof(int16_t));
}

// 日志补传: 每次最多传一条(最旧未发), 成功即标已发(4 字节写, 无擦除); 失败退避 2s。
static void waveDrainTick(){
  static uint32_t lastFail=0;
  if(!g_wlog.ok() || WiFi.status()!=WL_CONNECTED) return;
  if(lastFail && millis()-lastFail<2000) return;
  int s=g_wlog.oldestUnsent(); if(s<0) return;
  WaveLogHeader h;
  if(!g_wlog.readRecord(s, h, g_waveBuf, sizeof g_waveBuf)){   // 扫描时好、现在读坏: 不应发生, 跳过别卡死
    Serial.printf("[wlog] 扇区 %d 读取/CRC 失败, 跳过\n", s); g_wlog.markSent(s); return;
  }
  if(postWaveChunk(h.ev_ms, h.chunk_seq, h.t0_ms, h.n, h.fs, h.lsb_g, g_waveBuf)){ g_wlog.markSent(s); lastFail=0; }
  else lastFail=millis();
}

static void waveStreamTick(){
  if(!g_ws.active){
    if(g_evSeq==g_ws.seenSeq || !g_evStrongOk) return;
    int64_t evMs = idxToEpochMs(g_evStartIdx);
    if(!evMs) return;                                  // 未对时无法标识事件: 不传波形(摘要照常上报)
    g_ws.seenSeq=g_evSeq; g_ws.active=true; g_ws.ended=false; g_ws.chunkSeq=0; g_ws.lastFailMs=0;
    g_ws.evStartIdx=g_evStartIdx; g_ws.evMs=evMs;
    uint32_t from = g_evStartIdx - (uint32_t)(WAVE_PRE_S*FS);
    if((int32_t)(from - g_wave.oldest()) < 0) from = g_wave.oldest();
    g_ws.next=from;
    Serial.printf("[wave] 开始流式上报 ev=%lld, 触发前 %.1fs 起\n", (long long)evMs, (int32_t)(g_evStartIdx-from)/FS);
  }
  // 事件已结束(或已被下一个事件接替): 记下终点, 把余下的发完
  if(!g_ws.ended && (!g_evActive || g_evSeq!=g_ws.seenSeq)){ g_ws.ended=true; g_ws.endIdx=g_evEndIdx; }
  // 落盘/直传持续失败, 环里已没有这段数据: 放弃, 别卡住后续事件
  if(g_ws.ended && (int32_t)(g_wave.head()-g_ws.endIdx) > g_wave.capacity()){
    Serial.println("[wave] 写出落后过久, 放弃该事件余下波形"); g_ws.active=false; return;
  }
  if(!g_wlog.ok() && WiFi.status()!=WL_CONNECTED) return;          // 无日志分区时直传需要网络
  if(g_ws.lastFailMs && millis()-g_ws.lastFailMs < 2000) return;   // 失败退避 2s
  uint32_t limit = g_ws.ended ? g_ws.endIdx : g_wave.head();
  int32_t avail = (int32_t)(limit - g_ws.next);
  if(avail >= WAVE_CHUNK || (g_ws.ended && avail > 0)){
    int n = avail > WAVE_CHUNK ? WAVE_CHUNK : avail;
    int got = g_wave.read(g_ws.next, n, g_waveBuf);
    if(got < 0){                                        // 起点已被覆盖: 跳到环里最旧的, 中间一段丢失
      uint32_t o=g_wave.oldest();
      Serial.printf("[wave] 丢失 %ld 样本(上报落后于采样)\n", (long)(int32_t)(o-g_ws.next));
      g_ws.next=o; return;
    }
    if(got == 0) return;
    int64_t t0 = idxToEpochMs(g_ws.next);
    bool okk;
    if(g_wlog.ok()){
      okk = g_wlog.append(g_ws.evMs, g_ws.chunkSeq, t0, (uint16_t)got, (uint16_t)FS, (uint16_t)WAVE_LSB_PER_G, g_waveBuf) >= 0;
      if(!okk) Serial.println("[wlog] append 失败");
    } else {
      okk = postWaveChunk(g_ws.evMs, g_ws.chunkSeq, t0, got, (int)FS, WAVE_LSB_PER_G, g_waveBuf);
    }
    if(okk){ g_ws.next += (uint32_t)got; g_ws.chunkSeq++; g_ws.lastFailMs=0; }
    else g_ws.lastFailMs=millis();
  }
  if(g_ws.ended && (int32_t)(g_ws.next - g_ws.endIdx) >= 0){
    g_ws.active=false;
    Serial.printf("[wave] 事件 ev=%lld 波形写出完成, %lu 块(日志未发 %d)\n",
                  (long long)g_ws.evMs, (unsigned long)g_ws.chunkSeq, g_wlog.countUnsent());
  }
}

// ---------- 采样任务(core 0) ----------
void sampleTask(void*){
  const TickType_t period = pdMS_TO_TICKS(10);   // 100Hz
  TickType_t last = xTaskGetTickCount();
  int16_t rx, ry, rz;
  bool ledState=false; uint32_t lastBlink=0;
  uint32_t btnDownMs=0; bool btnLongFired=false;   // 顶部按钮长按检测
  bool prevActive=false;
  for(;;){
    // 安全点应用 loop 下发的新配置(临界区只复制结构体, set_config 在本核执行)
    if(g_cfgPending){
      portENTER_CRITICAL(&g_cfgMux);
      DetConfig c = g_pendingCfg; g_cfgPending = false;
      portEXIT_CRITICAL(&g_cfgMux);
      detector.set_config(c);
    }
    // 原始 int16 ADC 先入环(无损, 供波形上报), 再按库的同一换算(adc*aRes, 单位 g)喂检测器
    M5.IMU.getAccelAdc(&rx, &ry, &rz);
    g_wave.push(rx, ry, rz);
    const float aRes = M5.IMU.aRes;
    detector.push(rx*aRes*980.665f, ry*aRes*980.665f, rz*aRes*980.665f);
    // 事件状态发布(供 loop 在事件结束前就开始流式上报波形)
    bool act = detector.active();
    if(act && !prevActive){
      g_evStartIdx=(uint32_t)detector.event_start_idx(); g_evStrongOk=false;
      g_evSeq=g_evSeq+1; g_evActive=true;
    }
    if(act && !g_evStrongOk && detector.strong_span_s() >= detector.get_config().min_dur_s) g_evStrongOk=true;
    if(!act && prevActive){ g_evEndIdx=(uint32_t)detector.sample_idx(); g_evActive=false; }
    prevActive=act;
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
      // 起止样本序号(与环缓冲同一计数; loop 经时间锚折算 epoch ms)
      q.startIdx = (uint32_t)(ev.start_s*FS + 0.5);
      q.endIdx   = (uint32_t)(ev.end_s*FS + 0.5);
      Serial.printf("[event] dur=%.1fs strong=%.1fs peak=%.1fgal H=%.1f V=%.1f shindo=%.1f(%s) clip=%d\n",
                    ev.dur_s, ev.strong_s, ev.peak_gal, ev.peak_h_gal, ev.peak_v_gal,
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
  // 原始波形环: 60s×100Hz×6B=36KB, 堆就绪后分配(全局构造期分配会 boot-loop)。失败则缩到 30s 再试。
  uint32_t heapBefore = ESP.getFreeHeap();
  if(!g_wave.begin(WAVE_RING_S*(int)FS) && !g_wave.begin(WAVE_RING_S/2*(int)FS))
    Serial.println("[wave] 环缓冲分配失败, 本次不上报原始波形");
  Serial.printf("[wave] 环 %d 样本(%.0fs), 堆 %u -> %u B\n", g_wave.capacity(), g_wave.capacity()/FS, heapBefore, ESP.getFreeHeap());
  // 波形 flash 日志分区(partitions.csv 的 wavelog): 扫描恢复未发记录。没有该分区(老分区表)则退回直传。
  g_wlogIo.part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "wavelog");
  if(g_wlogIo.part && g_wlog.begin(&g_wlogIo, g_wlogIo.part->size))
    Serial.printf("[wlog] 分区 %u KB, %d 扇区: 未发 %d / 已发 %d / 空白 %d\n", (unsigned)(g_wlogIo.part->size/1024),
                  g_wlog.sectors(), g_wlog.countUnsent(), g_wlog.countSent(), g_wlog.countBlank());
  else Serial.println("[wlog] 无 wavelog 分区(老分区表), 波形改为直传、断网即丢");
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
  if(!g_anchorMs) refreshTimeAnchor();   // NTP 一对上时就建立样本序号<->epoch 锚
  // 1) 原始波形: 环 -> flash 日志(先于摘要处理: 事件的 ev 毫秒值由此确定, 摘要沿用同值); 日志 -> 服务器补传
  waveStreamTick();
  waveDrainTick();
  // 2) 取队列里的事件摘要 POST(失败转 NVS 缓冲)
  static QueuedEvent q;       // ~3.7KB, static 避免占 loop 栈
  if(g_evQ && xQueueReceive(g_evQ, &q, 0)==pdTRUE){
    static char body[8192];   // 含 300 点自适应包络, static 避免占 loop 栈
    // 事件起止 epoch ms: 同一时间锚折算; 若该事件已在流式上报, 直接沿用其 ev 值保证严格一致
    int64_t start_ms = (g_ws.evMs && g_ws.evStartIdx==q.startIdx) ? g_ws.evMs : idxToEpochMs(q.startIdx);
    int64_t end_ms   = idxToEpochMs(q.endIdx);
    buildEventBody(q.ev, start_ms, end_ms, body, sizeof(body), true);        // 完整(含包络)上报
    if(!postJson("", body)){
      static char cbody[1024];
      buildEventBody(q.ev, start_ms, end_ms, cbody, sizeof(cbody), false);   // 离线只存标量(无包络), 省 NVS
      bufferEvent(cbody);
    }
  }

  // 3) WiFi 维护; 每 15s: 拉远程配置 + 补发 + 心跳(带噪声/峰值/堆遥测) + 空闲时刷新时间锚
  static uint32_t lastWifi=0, lastTick=0, lastMaint=0;
  if(WiFi.status()!=WL_CONNECTED && millis()-lastWifi>5000){ lastWifi=millis(); wifiConnect(); }
  // 日志维护: 空闲时每 5s 擦一个已发/坏扇区(擦除会停采样核 ~45ms, 所以零散做、事件中不做), 给下次事件备好空白
  if(millis()-lastMaint>5000){ lastMaint=millis(); if(!g_evActive && !g_ws.active) g_wlog.maintain(1); }
  if(millis()-lastTick>15000){
    lastTick=millis();
    if(!g_evActive && !g_ws.active) refreshTimeAnchor();   // 事件/上报进行中不动锚, 保证同一事件内时刻一致
    if(WiFi.status()==WL_CONNECTED){
      fetchConfig();                              // 拉配置热更新
      flushBuffer();
      float peak = g_peakGal; g_peakGal = 0;      // 取走峰值(本窗口最大)
      const DetConfig& a = g_appliedCfg;          // 回报设备实际生效的配置(供面板确认下发到位)
      char hb[480]; snprintf(hb,sizeof(hb),
        "{\"mac\":\"%s\",\"fw\":\"%s\",\"rssi\":%d,\"uptime_ms\":%lu,"
        "\"noise_gal\":%.1f,\"peak_gal\":%.1f,\"heap\":%u,\"heap_min\":%u,\"wave_s\":%d,"
        "\"wlog_unsent\":%d,\"wlog_sectors\":%d,\"cfg\":{"
        "\"k\":%.1f,\"floor_min\":%.1f,\"holdoff_s\":%.1f,"
        "\"min_dur_s\":%.1f,\"led_min_s\":%.1f,\"min_keisoku\":%.2f,"
        "\"k_off\":%.1f,\"floor_off\":%.1f,\"tail_max_s\":%.0f}}",
        g_mac, FW_VER, WiFi.RSSI(), (unsigned long)millis(), g_noiseGal, peak,
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (int)(g_wave.capacity()/FS),
        g_wlog.countUnsent(), g_wlog.sectors(),
        a.k, a.floor_min, a.holdoff_s, a.min_dur_s, a.led_min_s, a.min_keisoku,
        a.k_off, a.floor_off, a.tail_max_s);
      if(postJson("/heartbeat", hb)) otaConfirmIfPending();   // 联网成功 -> 确认新固件
    }
    // OTA 待确认超时(5min 仍未联网成功): 重启触发回滚到旧固件
    if(g_otaPending && millis()-g_bootMs > 300000){
      Serial.println("[ota] 待确认超时, 重启回滚"); esp_restart();
    }
  }
  delay(5);
}
