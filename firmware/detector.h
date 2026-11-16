#pragma once
#include <cstdint>
#include "jma_stream.h"

// 包络格数上限。画满后格宽翻倍(相邻取max合并), 任意长度事件都能全程覆盖,
// 只是超长事件时间分辨率自适应变粗(100ms->200ms->400ms...)。
static const int EQ_ENV_CAP = 300;

// 一次震动事件的摘要。时间为相对秒(由样本计数/fs 推算)；幅度单位 gal。
struct EventSummary {
  double start_s, end_s, dur_s;       // dur_s = 全程震动跨度(起点→末次超过"收尾阈值", 不含 hold-off 尾巴)
  double strong_s;                    // 强晃跨度(起点→末次超过"触发阈值"); 录入门限 min_dur_s 按它判
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
  // 收尾用第二道(更低的)阈值 —— 起停迟滞。触发要越过 thr_on 防误报; 一旦触发, 只要还超过
  // thr_off = max(k_off*噪声底, floor_off) 就视为仍在震, 这样地震尾波(十几 gal 的余晃)不会被
  // 较高的触发阈值提前切掉。thr_off 自动不高于 thr_on。
  double k_off      = 2.0;    // 收尾阈值噪声倍数
  double floor_off  = 3.0;    // 收尾阈值下限(gal)
  double tail_max_s = 120.0;  // 强晃(超 thr_on)停止后最多再记录多久; 防止旁边持续走路让事件收不了尾
};

// 二阶 IIR 节(直接 II 转置型)。系数在构造时按 fs 用双线性变换算出(RBJ 公式), 无堆分配。
struct Biquad {
  double b0=1, b1=0, b2=0, a1=0, a2=0, z1=0, z2=0;
  static Biquad lowpass(double fs, double fc, double q);
  static Biquad highpass(double fs, double fc, double q);
  double step(double x){ double y=b0*x+z1; z1=b1*x-a1*y+z2; z2=b2*x-a2*y; return y; }
  void reset(){ z1=z2=0; }
};

// 检测频带: 0.5–10 Hz(二阶高通 + 四阶 Butterworth 低通)。地震动能量绝大部分在这一段;
// 敲击/关门/放重物等撞击是 10 Hz 以上的短脉冲。2026-11-16 误报 #39(撞击, 原始峰 25 gal)
// 10 Hz 以上能量占 87%, 过此滤波后峰值 6.6 gal; 真震 #36/#37 只占 3–4%, 峰值 13.7→12.8 / 19.3→19.3。
static const double EQ_BAND_LO_HZ = 0.5;
static const double EQ_BAND_HI_HZ = 10.0;

// 事件检测器：100Hz 推入三轴加速度(gal)，界定"震动事件"。
// 触发/收尾判定、噪声底、遥测峰值都用"检测频带内"的动态加速度; 事件的峰值/包络/震度仍用全频带(原始物理量)。
class Detector {
 public:
  Detector(double fs, const DetConfig& cfg);
  void begin();                              // setup 中调用(堆就绪后): 预构建流式震度 FIR 抽头
  void set_config(const DetConfig& cfg);     // 运行期热更新(仅在采样任务里调用, 无锁)
  DetConfig get_config() const { return cfg_; }
  bool push(float ax, float ay, float az);   // 返回当前是否震动中(瞬时)
  bool active() const { return active_; }
  bool led_on() const { return ledOn_; }     // 强晃跨度≥led_min 即亮; 强晃停 holdoff 即灭(记录可继续)
  bool poll_event(EventSummary& out);        // 有刚结束(且达门限)的事件则取出返回 true
  double noise_floor() const { return nf_; } // 遥测: 当前噪声底(gal, 检测频带内)
  double now_s() const { return (double)sampleIdx_/fs_; } // 当前样本时刻(与 EventSummary.start_s/end_s 同基准)
  uint64_t sample_idx() const { return sampleIdx_; }      // 已推入样本数(= 下一样本的绝对序号)
  // 进行中事件的起点样本序号 / 当前强晃跨度(秒)。供固件在事件结束前就开始流式上报原始波形:
  // 强晃跨度一达到 min_dur_s 就可确定该事件不会被时长门限丢弃。
  uint64_t event_start_idx() const { return evStart_; }
  double strong_span_s() const { return active_ ? (double)(lastAbove_-evStart_+1)/fs_ : 0.0; }
  float take_recent_peak();                  // 遥测: 自上次以来检测频带内 |动态| 峰值(与触发阈值同口径), 取走并清零
  // 噪声底估计: 每秒一块(检测频带 |b| 均值), 保留最近 NF_WIN 块, 取第 NF_PCT 分位。
  // 分位数而非均值/EMA: 地震 P 波先于 S 波几秒到几十秒抵达、幅度小却已高于噪声, 旧的 5s EMA
  // 会被它抬起数倍, 触发阈值随之飙高, S 波触发后尾波很快"低于阈值"被提前收尾(2026-08-23 实震:
  // 噪声底 0.9→5.7gal, 阈值 34gal, 12~31gal 的尾波全被切掉)。分位数只要窗口里 (1-NF_PCT) 的时间
  // 是安静的就不受污染: 120s 窗 + 20 分位可容忍 96s 前驱振动; 事件期间冻结不学习。
  static const int NF_WIN = 120;
  static constexpr double NF_PCT = 0.20;
 private:
  double fs_;
  DetConfig cfg_;
  JmaStream jma_;                                   // 流式計測震度(取代旧 2048 FFT 缓冲)
  double gx_=0, gy_=0, gz_=0; bool gInit_=false;   // 重力 EMA(始终更新)
  static const int BAND_SECTIONS = 3;               // 高通 1 节 + 低通 2 节
  Biquad band_[3][BAND_SECTIONS];                   // 每轴一组检测频带滤波器
  double nf_=0;                                     // 噪声底(最近 NF_WIN 秒检测频带 |b| 块均值的 NF_PCT 分位)
  float nfRing_[NF_WIN]; int nfN_=0, nfPos_=0;     // 块均值环
  double nfBlockSum_=0; int nfBlockCnt_=0;          // 当前块累加
  float recentPeak_=0;                             // 遥测滚动峰值
  bool active_=false, ledOn_=false; int belowCnt_=0, aboveCnt_=0;
  uint64_t sampleIdx_=0, evStart_=0, lastAbove_=0, lastAboveOff_=0;
  double pk_=0, pkH_=0, pkV_=0, sumSq_=0; uint64_t cnt_=0; bool clip_=false;
  // 自适应包络: 当前每格样本数 envSpb_(画满翻倍), 各格记峰值
  float envTot_[EQ_ENV_CAP], envH_[EQ_ENV_CAP], envV_[EQ_ENV_CAP]; int envN_=0;
  int envSpb_=0;
  bool pending_=false; EventSummary done_{};
  void resetEvent();
  void nfPushBlock(float v);   // 一块均值入环并重算分位
  double bandStep(int axis, double x);
};
