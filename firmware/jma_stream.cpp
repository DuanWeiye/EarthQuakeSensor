#include "jma_stream.h"
#include "jma_intensity.h"
#include <cmath>
#include <vector>
#include <cstring>

JmaStream::JmaStream(double fs, int half_m) : fs_(fs) {
  M_ = half_m; if(M_ < 1) M_ = 1; if(M_ > MAX_M) M_ = MAX_M;
  L_ = 2*M_ + 1;
  K_ = (int)std::lround(0.3 * fs_); if(K_ < 1) K_ = 1;
  if(K_ > (int)(sizeof(heap_)/sizeof(heap_[0]))) K_ = sizeof(heap_)/sizeof(heap_[0]);
  // 不在此构建抽头: 全局静态构造期堆未必就绪, std::vector 分配会 bad_alloc->abort。
  // 延后到首次 reset()(运行期, 堆已就绪); 设备侧由 setup() 预热。
  built_ = false; wpos_ = 0; heapN_ = 0;
}

// 由 G(f) 逆FFT 得时域脉冲响应, 取对称 FIR 抽头。G 实偶 -> h 实偶 -> 零相位对称。
void JmaStream::buildTaps(){
  const int NF = 4096;                       // 频率栅格(分辨率 fs/4096 ~0.024Hz, 足够刻画0.5Hz低切)
  std::vector<double> re(NF, 0.0), im(NF, 0.0);
  for(int k=0; k<=NF/2; k++){
    double f = (double)k * fs_ / NF;
    double g = jma_filter_gain(f);           // 已验证的官方滤波增益
    re[k] = g;
    if(k>0 && k<NF/2) re[NF-k] = g;          // 厄米对称(实偶谱)
  }
  fft_radix2(re.data(), im.data(), NF, true);  // 逆变换(内部已除 NF) -> re[d]=h[d]
  // 展开成对称抽头: 中心 cf_[M_]=h[0], cf_[M_±d]=h[d]
  for(int j=0; j<L_; j++){
    int d = std::abs(j - M_);                // 0..M_ (< NF/2, 无混叠)
    cf_[j] = re[d];
  }
}

void JmaStream::reset(){
  if(!built_){ buildTaps(); built_ = true; }     // 惰性构建(运行期堆就绪)
  std::memset(dx_, 0, sizeof(float)*L_);
  std::memset(dy_, 0, sizeof(float)*L_);
  std::memset(dz_, 0, sizeof(float)*L_);
  wpos_ = 0;
  heapN_ = 0;
}

void JmaStream::heapPush(double v){
  if(heapN_ < K_){                            // 未满: 插入并上浮(最小堆)
    int i = heapN_++;
    heap_[i] = v;
    while(i>0){ int p=(i-1)/2; if(heap_[p]<=heap_[i]) break; std::swap(heap_[p],heap_[i]); i=p; }
  } else if(v > heap_[0]){                     // 已满: 比当前最小大才替换堆顶并下沉
    heap_[0] = v;
    int i=0;
    for(;;){
      int l=2*i+1, r=2*i+2, s=i;
      if(l<heapN_ && heap_[l]<heap_[s]) s=l;
      if(r<heapN_ && heap_[r]<heap_[s]) s=r;
      if(s==i) break;
      std::swap(heap_[s],heap_[i]); i=s;
    }
  }
}

// 对延迟线做一次因果对称FIR(等价零相位滤波延迟M), 三轴合成幅度入堆。
void JmaStream::emit(){
  double yx=0, yy=0, yz=0;
  // y = Σ cf_[j] * x[wpos_-1-j]  (wpos_ 指向"下一个写入位", 最新样本在 wpos_-1)
  int idx = wpos_ - 1; if(idx < 0) idx += L_;
  for(int j=0; j<L_; j++){
    double c = cf_[j];
    yx += c * dx_[idx];
    yy += c * dy_[idx];
    yz += c * dz_[idx];
    if(--idx < 0) idx += L_;
  }
  heapPush(std::sqrt(yx*yx + yy*yy + yz*yz));
}

void JmaStream::push(double dx, double dy, double dz){
  dx_[wpos_] = (float)dx; dy_[wpos_] = (float)dy; dz_[wpos_] = (float)dz;
  if(++wpos_ >= L_) wpos_ = 0;
  emit();
}

void JmaStream::finish(){
  for(int t=0; t<M_; t++) push(0.0, 0.0, 0.0);   // 补零冲刷, 产出最后 M 个滤波样本
}

double JmaStream::a0() const {
  if(heapN_ < K_) return 0.0;                 // 不足 0.3s 有效样本
  return heap_[0];                            // 最小堆顶 = 第K大
}

double JmaStream::keisoku() const {
  double a = a0();
  if(a <= 0.0) return 0.0;
  return round_keisoku(2.0*std::log10(a) + 0.94);
}
