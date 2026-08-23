#include "detector.h"
#include "jma_intensity.h"
#include <cmath>
#include <algorithm>

Detector::Detector(double fs, const DetConfig& cfg) : fs_(fs), cfg_(cfg), jma_(fs) {}

// 预热: 在 setup()(堆已就绪)里构建流式震度的 FIR 抽头, 避免全局静态构造期分配崩溃,
// 也避免首个事件在采样核现场构建造成时序抖动。
void Detector::begin(){ jma_.reset(); }

void Detector::set_config(const DetConfig& cfg){ cfg_ = cfg; }

// 一块(1s)|d| 均值入环, 重算 NF_PCT 分位作为噪声底。环最多 NF_WIN 块, 每秒一次 nth_element, 开销可忽略。
void Detector::nfPushBlock(float v){
  nfRing_[nfPos_] = v; nfPos_ = (nfPos_+1) % NF_WIN; if(nfN_<NF_WIN) nfN_++;
  float tmp[NF_WIN]; for(int i=0;i<nfN_;i++) tmp[i]=nfRing_[i];
  int k = (int)(NF_PCT*(nfN_-1) + 0.5); if(k<0) k=0; if(k>=nfN_) k=nfN_-1;
  std::nth_element(tmp, tmp+k, tmp+nfN_);
  nf_ = tmp[k];
}

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
  // 噪声底: 非事件期按 1s 块累加 |d| 均值入环取分位(事件期冻结, 块累加也清零不让震动混入)
  if(!active_){
    nfBlockSum_+=mag; nfBlockCnt_++;
    if(nfBlockCnt_ >= (int)(fs_+0.5)){ nfPushBlock((float)(nfBlockSum_/nfBlockCnt_)); nfBlockSum_=0; nfBlockCnt_=0; }
  } else { nfBlockSum_=0; nfBlockCnt_=0; }
  // 起停迟滞: 触发阈值 thr_on 高(防误报), 收尾阈值 thr_off 低(留住尾波); thr_off 不高于 thr_on
  double thrOn =std::max(cfg_.k*nf_, cfg_.floor_min);
  double thrOff=std::min(thrOn, std::max(cfg_.k_off*nf_, cfg_.floor_off));
  bool over = mag>thrOn;
  bool overOff = mag>thrOff;
  if(over){
    aboveCnt_++;
    if(!active_ && aboveCnt_>=3){            // 去抖: 连续 3 样本超阈
      active_=true; evStart_=sampleIdx_-aboveCnt_+1;
      lastAbove_=lastAboveOff_=sampleIdx_; belowCnt_=0;
      resetEvent();
    }
  } else aboveCnt_=0;
  if(active_){
    if(over) lastAbove_=sampleIdx_;                 // 末次超触发阈值 -> 强晃跨度
    if(overOff){ lastAboveOff_=sampleIdx_; belowCnt_=0; } else belowCnt_++;   // 末次超收尾阈值 -> 全程跨度
    // LED 跟"强晃": 跨度达 led_min 亮; 强晃停了 holdoff 就灭(记录仍可在尾波里继续, 避免红灯闪几分钟像故障)
    uint64_t holdSamples = (uint64_t)(cfg_.holdoff_s*fs_);
    if(!ledOn_ && (lastAbove_-evStart_+1) >= (uint64_t)(cfg_.led_min_s*fs_+0.5)) ledOn_=true;
    if(ledOn_ && sampleIdx_-lastAbove_ >= holdSamples) ledOn_=false;
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
    // 收尾: 低于收尾阈值持续 holdoff; 或强晃停止已超 tail_max(兜底, 防持续弱扰动拖着不结束)
    bool tailTimeout = (sampleIdx_-lastAbove_) >= (uint64_t)(cfg_.tail_max_s*fs_);
    if((uint64_t)belowCnt_ >= holdSamples || tailTimeout){
      double strong=(double)(lastAbove_-evStart_+1)/fs_;
      double dur=(double)(lastAboveOff_-evStart_+1)/fs_;
      active_=false; belowCnt_=0; ledOn_=false;      // 事件结束, LED 灭
      if(strong < cfg_.min_dur_s) return active_;    // 强晃时长不够(轻碰), 丢弃
      jma_.finish();                                 // 冲刷 FIR 尾部
      double keis = jma_.keisoku();
      if(keis < cfg_.min_keisoku) return active_;    // 震度不够(走路等小扰动), 丢弃
      EventSummary s{};
      s.start_s=(double)evStart_/fs_;
      s.end_s=(double)lastAboveOff_/fs_;
      s.dur_s=dur; s.strong_s=strong;
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
