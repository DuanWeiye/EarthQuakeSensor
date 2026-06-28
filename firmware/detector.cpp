#include "detector.h"
#include "jma_intensity.h"
#include <cmath>
#include <algorithm>

Detector::Detector(double fs, const DetConfig& cfg) : fs_(fs), cfg_(cfg), jma_(fs) {}

// 预热: 在 setup()(堆已就绪)里构建流式震度的 FIR 抽头, 避免全局静态构造期分配崩溃,
// 也避免首个事件在采样核现场构建造成时序抖动。
void Detector::begin(){ jma_.reset(); }

void Detector::set_config(const DetConfig& cfg){ cfg_ = cfg; }

float Detector::take_recent_peak(){ float p=recentPeak_; recentPeak_=0; return p; }

void Detector::resetEvent(){
  pk_=pkH_=pkV_=sumSq_=0; cnt_=0; clip_=false;
  envN_=0; ledOn_=false;
  envSpb_ = (int)(fs_*0.1 + 0.5); if(envSpb_<1) envSpb_=1;   // 起步 100ms/格
  for(int i=0;i<EQ_ENV_CAP;i++){ envTot_[i]=envH_[i]=envV_[i]=0; }
  jma_.reset();                                              // 开新事件的流式震度
}

bool Detector::push(float ax,float ay,float az){
  sampleIdx_++;
  // 重力慢 EMA(时间常数 ~1.6s): α=1/(fs*1.6)。始终更新(不冻结), 移位后能重收敛、事件才能结束。
  double a=1.0/(fs_*1.6);
  if(!gInit_){ gx_=ax;gy_=ay;gz_=az;gInit_=true; }
  gx_+=a*(ax-gx_); gy_+=a*(ay-gy_); gz_+=a*(az-gz_);
  double dx=ax-gx_, dy=ay-gy_, dz=az-gz_;
  double mag=std::sqrt(dx*dx+dy*dy+dz*dz);
  if(mag>recentPeak_) recentPeak_=(float)mag;        // 遥测: 滚动峰值(供贴电视后看走路多大)
  double thr=std::max(cfg_.k*nf_, cfg_.floor_min);
  bool over = mag>thr;
  // 噪声底只在非事件时缓慢学习
  if(!active_){ double b=1.0/(fs_*5.0); nf_+=b*(mag-nf_); }
  if(over){
    aboveCnt_++; belowCnt_=0;
    if(!active_ && aboveCnt_>=3){            // 去抖: 连续 3 样本超阈
      active_=true; evStart_=sampleIdx_-aboveCnt_+1;
      resetEvent();
    }
  } else { belowCnt_++; aboveCnt_=0; }
  if(active_){
    if(over) lastAbove_=sampleIdx_;                 // 记最后一次超阈, 用于真实时长
    // LED: 用"持续晃动跨度"(末次超阈-起点)判定, 达 led_min 即亮(不含 hold-off 静默期)
    if(!ledOn_ && (lastAbove_-evStart_+1) >= (uint64_t)(cfg_.led_min_s*fs_+0.5)) ledOn_=true;
    // 水平/垂直分解: 用重力单位向量
    double gn=std::sqrt(gx_*gx_+gy_*gy_+gz_*gz_); if(gn<1e-6) gn=1;
    double ev=(dx*gx_+dy*gy_+dz*gz_)/gn;             // 垂直分量(沿重力)
    double hx=dx-ev*gx_/gn, hy=dy-ev*gy_/gn, hz=dz-ev*gz_/gn;
    double hmag=std::sqrt(hx*hx+hy*hy+hz*hz);
    pk_=std::max(pk_,mag); pkH_=std::max(pkH_,hmag); pkV_=std::max(pkV_,std::fabs(ev));
    sumSq_+=mag*mag; cnt_++;
    if(std::fabs(ax)>=4.0*980.665 || std::fabs(ay)>=4.0*980.665 || std::fabs(az)>=4.0*980.665) clip_=true;
    jma_.push(dx,dy,dz);                            // 流式喂入(去重力后的动态加速度)
    // 自适应包络: 当前格宽 envSpb_ 样本; 画满 EQ_ENV_CAP 格则格宽翻倍(相邻取max合并)
    uint64_t rel = sampleIdx_ - evStart_;
    int bin = (int)(rel / (uint64_t)envSpb_);
    while(bin >= EQ_ENV_CAP){
      int half = (EQ_ENV_CAP + 1)/2;
      for(int i=0;i<half;i++){
        int p=2*i, q=2*i+1;
        envTot_[i] = (q<EQ_ENV_CAP)? std::max(envTot_[p],envTot_[q]) : envTot_[p];
        envH_[i]   = (q<EQ_ENV_CAP)? std::max(envH_[p],envH_[q])     : envH_[p];
        envV_[i]   = (q<EQ_ENV_CAP)? std::max(envV_[p],envV_[q])     : envV_[p];
      }
      for(int i=half;i<EQ_ENV_CAP;i++){ envTot_[i]=envH_[i]=envV_[i]=0; }
      envSpb_ *= 2; envN_ = half;
      bin = (int)(rel / (uint64_t)envSpb_);
    }
    if(bin+1>envN_) envN_=bin+1;
    if(mag>envTot_[bin])        envTot_[bin]=(float)mag;
    if(hmag>envH_[bin])         envH_[bin]=(float)hmag;
    if(std::fabs(ev)>envV_[bin]) envV_[bin]=(float)std::fabs(ev);
    if(belowCnt_ >= (int)(cfg_.holdoff_s*fs_)){      // hold-off 后收尾
      double dur=(double)(lastAbove_-evStart_+1)/fs_;
      active_=false; belowCnt_=0; ledOn_=false;      // 事件结束, LED 灭
      if(dur < cfg_.min_dur_s) return active_;       // 时长不够(轻碰), 丢弃
      jma_.finish();                                 // 冲刷 FIR 尾部
      double keis = jma_.keisoku();
      if(keis < cfg_.min_keisoku) return active_;    // 震度不够(走路等小扰动), 丢弃
      EventSummary s{};
      s.start_s=(double)evStart_/fs_;
      s.end_s=(double)lastAbove_/fs_;
      s.dur_s=dur;
      s.peak_gal=pk_; s.peak_h_gal=pkH_; s.peak_v_gal=pkV_;
      s.rms_gal= cnt_? std::sqrt(sumSq_/cnt_):0.0;
      s.keisoku=keis;
      s.clipped=clip_;
      s.env_dt_ms = (double)envSpb_ * 1000.0 / fs_;  // 当前格宽对应的毫秒数
      s.env_n = envN_>EQ_ENV_CAP?EQ_ENV_CAP:envN_;
      for(int i=0;i<s.env_n;i++){ s.env_tot[i]=envTot_[i]; s.env_h[i]=envH_[i]; s.env_v[i]=envV_[i]; }
      done_=s; pending_=true;
    }
  }
  return active_;
}

bool Detector::poll_event(EventSummary& out){
  if(!pending_) return false;
  out=done_; pending_=false; return true;
}
