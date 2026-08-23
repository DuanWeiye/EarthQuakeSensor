#include "wavering.h"
#include <cstdlib>
#include <cstring>

bool WaveRing::begin(int capSamples){
  if(buf_){ std::free(buf_); buf_=nullptr; }
  cap_ = 0; head_ = 0;
  if(capSamples <= 0) return false;
  buf_ = (int16_t*)std::malloc((size_t)capSamples * 3 * sizeof(int16_t));
  if(!buf_) return false;
  cap_ = capSamples;
  return true;
}

void WaveRing::push(int16_t x, int16_t y, int16_t z){
  if(!buf_) return;
  uint32_t h = __atomic_load_n(&head_, __ATOMIC_RELAXED);
  int16_t* p = buf_ + (size_t)(h % (uint32_t)cap_) * 3;
  p[0]=x; p[1]=y; p[2]=z;
  __atomic_store_n(&head_, h+1, __ATOMIC_RELEASE);   // 数据落地后再公布序号
}

uint32_t WaveRing::head() const { return __atomic_load_n(&head_, __ATOMIC_ACQUIRE); }

uint32_t WaveRing::oldest() const {
  uint32_t h = head();
  return h > (uint32_t)cap_ ? h - (uint32_t)cap_ : 0;
}

int WaveRing::available(uint32_t from) const {
  int32_t d = (int32_t)(head() - from);
  return d > 0 ? d : 0;
}

int WaveRing::read(uint32_t from, int n, int16_t* out) const {
  if(!buf_ || n <= 0) return 0;
  uint32_t h = head();
  int32_t avail = (int32_t)(h - from);
  if(avail <= 0) return 0;
  if(avail > cap_) return -1;                         // from 已被覆盖
  if(n > avail) n = avail;
  for(int i=0;i<n;i++){
    const int16_t* p = buf_ + (size_t)((from + (uint32_t)i) % (uint32_t)cap_) * 3;
    out[3*i]=p[0]; out[3*i+1]=p[1]; out[3*i+2]=p[2];
  }
  // 拷贝期间生产者可能追上来覆盖了区段起点: 事后核对, 不一致则作废
  uint32_t h2 = head();
  if((int32_t)(h2 - from) > cap_) return -1;
  return n;
}
