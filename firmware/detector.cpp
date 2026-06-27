#include "detector.h"
#include "jma_intensity.h"
#include <cmath>
#include <algorithm>

Detector::Detector(double fs, const DetConfig& cfg) : fs_(fs), cfg_(cfg) {}

void Detector::set_config(const DetConfig& cfg){ cfg_ = cfg; }

float Detector::take_recent_peak(){ float p=recentPeak_; recentPeak_=0; return p; }

void Detector::pushBuf(float ax,float ay,float az){
  int i=(head_+len_)%CAP;
  if(len_<CAP){ bx_[i]=ax;by_[i]=ay;bz_[i]=az; len_++; }
  else { bx_[head_]=ax;by_[head_]=ay;bz_[head_]=az; head_=(head_+1)%CAP; }
}

double Detector::finalizeKeisoku(){
  // 取缓冲尾部 2 的幂个样本(最多 2048), 顺序展开后算计测震度
  int n=1; while(n*2<=len_ && n<CAP) n<<=1;        // 最大不超过 len_ 的 2 的幂
  if(n<256) return 0.0;                            // 样本太少不算(<2.56s)
  static float ax[CAP],ay[CAP],az[CAP];
  int startOff=len_-n;
  for(int j=0;j<n;j++){ int idx=(head_+startOff+j)%CAP; ax[j]=bx_[idx];ay[j]=by_[idx];az[j]=bz_[idx]; }
  return compute_keisoku_shindo(ax,ay,az,n,fs_);
}

void Detector::resetEvent(){
  pk_=pkH_=pkV_=sumSq_=0; cnt_=0; clip_=false; len_=0; head_=0;
  envN_=0; ledOn_=false;
  for(int i=0;i<EQ_ENV_CAP;i++){ envTot_[i]=envH_[i]=envV_[i]=0; }
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
    // 100ms 包络: 每 bin 取峰值
    int spb = (int)(fs_*0.1 + 0.5);
    int bin = (int)((sampleIdx_ - evStart_) / (uint64_t)(spb>0?spb:1));
    if(bin>=0 && bin<EQ_ENV_CAP){
      if(bin+1>envN_) envN_=bin+1;
      if(mag>envTot_[bin])        envTot_[bin]=(float)mag;
      if(hmag>envH_[bin])         envH_[bin]=(float)hmag;
      if(std::fabs(ev)>envV_[bin]) envV_[bin]=(float)std::fabs(ev);
    }
    pushBuf((float)dx,(float)dy,(float)dz);
    if(belowCnt_ >= (int)(cfg_.holdoff_s*fs_)){      // hold-off 后收尾
      double dur=(double)(lastAbove_-evStart_+1)/fs_;
      active_=false; belowCnt_=0; ledOn_=false;      // 事件结束, LED 灭
      if(dur < cfg_.min_dur_s) return active_;       // 时长不够(轻碰), 丢弃
      double keis = finalizeKeisoku();
      if(keis < cfg_.min_keisoku) return active_;    // 震度不够(走路等小扰动), 丢弃
      EventSummary s{};
      s.start_s=(double)evStart_/fs_;
      s.end_s=(double)lastAbove_/fs_;
      s.dur_s=dur;
      s.peak_gal=pk_; s.peak_h_gal=pkH_; s.peak_v_gal=pkV_;
      s.rms_gal= cnt_? std::sqrt(sumSq_/cnt_):0.0;
      s.keisoku=keis;
      s.clipped=clip_;
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
