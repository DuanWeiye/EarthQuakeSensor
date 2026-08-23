#include "wavelog.h"
#include <cstring>

// 标准 CRC32(IEEE 802.3, 反射, 初值/终值异或 0xFFFFFFFF), 与 Python zlib.crc32 一致, 服务端可复核。
// 流式更新接口: 扫描时分段读取分段算, 不需要 4KB 栈缓冲。
static uint32_t crcUpdate(uint32_t c, const uint8_t* p, size_t len){
  static uint32_t table[256]; static bool init=false;
  if(!init){ for(uint32_t i=0;i<256;i++){ uint32_t x=i; for(int k=0;k<8;k++) x=(x&1)?0xEDB88320u^(x>>1):x>>1; table[i]=x; } init=true; }
  for(size_t i=0;i<len;i++) c = table[(c^p[i])&0xFF] ^ (c>>8);
  return c;
}
uint32_t WaveLog::crc32(const void* data, size_t len){
  return crcUpdate(0xFFFFFFFFu, (const uint8_t*)data, len) ^ 0xFFFFFFFFu;
}

void WaveLog::scanSector(int s){
  WaveLogHeader h; uint32_t off=(uint32_t)s*SECTOR;
  if(!io_->read(off,&h,sizeof h)){ state_[s]=BAD; rid_[s]=0; return; }
  if(h.magic==0xFFFFFFFFu && h.rid==0xFFFFFFFFu && h.crc==0xFFFFFFFFu){ state_[s]=BLANK; rid_[s]=0; return; }
  if(h.magic!=WAVELOG_MAGIC || h.n==0 || (size_t)h.n*6>(size_t)MAX_DATA){ state_[s]=BAD; rid_[s]=0; return; }
  // 校验数据 crc(写到一半断电: 头完整数据残缺, 或反之), 512B 分段读
  uint8_t buf[512]; uint32_t c=0xFFFFFFFFu; size_t left=(size_t)h.n*6, pos=0;
  while(left){
    size_t n = left<sizeof buf ? left : sizeof buf;
    if(!io_->read(off+HDR+pos, buf, n)){ state_[s]=BAD; rid_[s]=0; return; }
    c = crcUpdate(c, buf, n); pos+=n; left-=n;
  }
  if((c^0xFFFFFFFFu)!=h.crc){ state_[s]=BAD; rid_[s]=0; return; }
  state_[s] = (h.sent==0xFFFFFFFFu) ? UNSENT : SENT;
  rid_[s] = h.rid;
}

bool WaveLog::begin(FlashIO* io, uint32_t sizeBytes){
  io_=io; nsec_=(int)(sizeBytes/SECTOR); if(nsec_>MAX_SECTORS) nsec_=MAX_SECTORS;
  if(!io_ || nsec_<=0){ io_=nullptr; nsec_=0; return false; }
  uint32_t maxRid=0; int maxS=-1; bool any=false;
  for(int s=0;s<nsec_;s++){
    scanSector(s);
    if(state_[s]==UNSENT || state_[s]==SENT){
      if(!any || (int32_t)(rid_[s]-maxRid)>0){ maxRid=rid_[s]; maxS=s; any=true; }
    }
  }
  wp_ = any ? (maxS+1)%nsec_ : 0;
  nextRid_ = any ? maxRid+1 : 1;
  // 开机一次性把已发/坏扇区擦干净(此时采样尚未开始, 停顿无所谓): 首次启用时分区里是旧分区表的残留数据(全"坏"),
  // 不擦的话事件中每块都要就地擦; 上次事件的已发记录也一并回收。只留未发 + 空白。
  bootErased_ = maintain(nsec_);
  return true;
}

int WaveLog::append(int64_t ev_ms, uint32_t chunk_seq, int64_t t0_ms, uint16_t n, uint16_t fs, uint16_t lsb_g,
                    const int16_t* data){
  if(!ok() || n==0 || (size_t)n*6>(size_t)MAX_DATA) return -1;
  int s=wp_; uint32_t off=(uint32_t)s*SECTOR;
  if(state_[s]!=BLANK){                                  // 没有预擦好的扇区: 就地擦(会停采样核 ~45ms, 尽量少发生)
    if(!io_->eraseSector(off)) return -1;
    state_[s]=BLANK; inEventErases_++;
  }
  size_t bytes=(size_t)n*6;
  WaveLogHeader h; memset(&h, 0xFF, sizeof h);          // 未用字节保持 0xFF(擦除态)
  h.magic=WAVELOG_MAGIC; h.rid=nextRid_; h.ev_ms=ev_ms; h.chunk_seq=chunk_seq; h.t0_ms=t0_ms;
  h.n=n; h.fs=fs; h.lsb_g=lsb_g; h.reserved=0xFFFF; h.crc=crc32(data,bytes); h.sent=0xFFFFFFFFu;
  // 先数据后头: 头完整即数据完整(断电只会留下"无头"或"头+坏crc"的扇区, 扫描时判坏)
  if(!io_->write(off+HDR, data, bytes)) return -1;
  if(!io_->write(off, &h, sizeof h)) return -1;
  state_[s]=UNSENT; rid_[s]=nextRid_++;
  wp_=(s+1)%nsec_;
  return s;
}

int WaveLog::oldestUnsent() const {
  int best=-1; uint32_t bestRid=0;
  for(int s=0;s<nsec_;s++) if(state_[s]==UNSENT && (best<0 || (int32_t)(rid_[s]-bestRid)<0)){ best=s; bestRid=rid_[s]; }
  return best;
}

bool WaveLog::readRecord(int sector, WaveLogHeader& h, int16_t* data, size_t dataCapBytes) const {
  if(!ok() || sector<0 || sector>=nsec_) return false;
  uint32_t off=(uint32_t)sector*SECTOR;
  if(!io_->read(off,&h,sizeof h)) return false;
  if(h.magic!=WAVELOG_MAGIC) return false;
  size_t bytes=(size_t)h.n*6; if(bytes==0 || bytes>dataCapBytes || bytes>(size_t)MAX_DATA) return false;
  if(!io_->read(off+HDR, data, bytes)) return false;
  return crc32(data,bytes)==h.crc;
}

bool WaveLog::markSent(int sector){
  if(!ok() || sector<0 || sector>=nsec_ || state_[sector]!=UNSENT) return false;
  uint32_t zero=0;
  if(!io_->write((uint32_t)sector*SECTOR+SENT_OFF, &zero, 4)) return false;
  state_[sector]=SENT; return true;
}

int WaveLog::maintain(int maxErase){
  if(!ok()) return 0;
  int done=0;
  for(int i=0;i<nsec_ && done<maxErase;i++){
    int s=(wp_+i)%nsec_;                                  // 从写指针往前擦: 下次写优先碰到空白
    if(state_[s]==SENT || state_[s]==BAD){
      if(io_->eraseSector((uint32_t)s*SECTOR)){ state_[s]=BLANK; rid_[s]=0; done++; }
    }
  }
  return done;
}

int WaveLog::countUnsent() const { int c=0; for(int s=0;s<nsec_;s++) if(state_[s]==UNSENT) c++; return c; }
int WaveLog::countBlank()  const { int c=0; for(int s=0;s<nsec_;s++) if(state_[s]==BLANK)  c++; return c; }
int WaveLog::countSent()   const { int c=0; for(int s=0;s<nsec_;s++) if(state_[s]==SENT)   c++; return c; }
