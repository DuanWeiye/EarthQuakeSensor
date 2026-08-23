#pragma once
#include <cstdint>
#include "jma_stream.h"

// 包络格数上限。画满后格宽翻倍(相邻取max合并), 任意长度事件都能全程覆盖,
// 只是超长事件时间分辨率自适应变粗(100ms->200ms->400ms...)。
static const int EQ_ENV_CAP = 300;

// 一次震动事件的摘要。时间为相对秒(由样本计数/fs 推算)；幅度单位 gal。
struct EventSummary {
  double start_s, end_s, dur_s;       // dur_s = 超阈震动跨度(不含 hold-off 尾巴)
  double peak_gal, peak_h_gal, peak_v_gal, rms_gal, keisoku;
  bool clipped;                       // ±4g 触顶
  int env_n;                          // 包络点数
  double env_dt_ms;                   // 每格时间宽度(ms); 自适应, 超长事件会 >100
  float env_tot[EQ_ENV_CAP];
  float env_h[EQ_ENV_CAP];
  float env_v[EQ_ENV_CAP];
};

// 可热更新的检测参数(全部能通过网络远程调; 设备贴上去后只靠网络改这些)
struct DetConfig {
  double k          = 6.0;    // 噪声倍数: 阈值 = max(k*噪声底, floor_min)
  double floor_min  = 30.0;   // 触发下限(gal)
  double holdoff_s  = 1.5;    // 低于阈值持续此时长则事件收尾
  double min_dur_s  = 2.0;    // 录入门限: 震动跨度 < 此值不入库(滤轻碰)
  double led_min_s  = 0.5;    // LED 门限: 震动跨度 ≥ 此值才亮灯
  double min_keisoku= 0.5;    // 录入门限: 计测震度 < 此值不入库(滤走路等小扰动)
};

// 事件检测器：100Hz 推入三轴加速度(gal)，界定"震动事件"。
class Detector {
 public:
  Detector(double fs, const DetConfig& cfg);
  void begin();                              // setup 中调用(堆就绪后): 预构建流式震度 FIR 抽头
  void set_config(const DetConfig& cfg);     // 运行期热更新(仅在采样任务里调用, 无锁)
  DetConfig get_config() const { return cfg_; }
  bool push(float ax, float ay, float az);   // 返回当前是否震动中(瞬时)
  bool active() const { return active_; }
  bool led_on() const { return ledOn_; }     // 持续晃动跨度≥led_min 即亮
  bool poll_event(EventSummary& out);        // 有刚结束(且达门限)的事件则取出返回 true
  double noise_floor() const { return nf_; } // 遥测: 当前噪声底(gal)
  double now_s() const { return (double)sampleIdx_/fs_; } // 当前样本时刻(与 EventSummary.start_s/end_s 同基准)
  float take_recent_peak();                  // 遥测: 自上次以来 |动态| 峰值, 取走并清零
 private:
  double fs_;
  DetConfig cfg_;
  JmaStream jma_;                                   // 流式計測震度(取代旧 2048 FFT 缓冲)
  double gx_=0, gy_=0, gz_=0; bool gInit_=false;   // 重力 EMA(始终更新)
  double nf_=0;                                     // 噪声底(|d| 慢 EMA)
  float recentPeak_=0;                             // 遥测滚动峰值
  bool active_=false, ledOn_=false; int belowCnt_=0, aboveCnt_=0;
  uint64_t sampleIdx_=0, evStart_=0, lastAbove_=0;
  double pk_=0, pkH_=0, pkV_=0, sumSq_=0; uint64_t cnt_=0; bool clip_=false;
  // 自适应包络: 当前每格样本数 envSpb_(画满翻倍), 各格记峰值
  float envTot_[EQ_ENV_CAP], envH_[EQ_ENV_CAP], envV_[EQ_ENV_CAP]; int envN_=0;
  int envSpb_=0;
  bool pending_=false; EventSummary done_{};
  void resetEvent();
};
