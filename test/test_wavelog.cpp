#include "../firmware/wavelog.h"
#include <cstdio>
#include <cstring>
#include <vector>
static int fails=0;
static void check(const char* n, bool c){ std::printf(c?"ok   %s\n":"FAIL %s\n", n); if(!c) fails++; }

// 内存 NOR 模拟: 擦除=0xFF, 写只能 1→0(违反即失败), 可注入"写到第 k 字节断电"
struct SimFlash : FlashIO {
  std::vector<uint8_t> mem; int erases=0, writes=0; long dieAfterBytes=-1;
  explicit SimFlash(size_t n): mem(n,0xFF) {}
  bool read(uint32_t off, void* buf, size_t len) override { if(off+len>mem.size()) return false; memcpy(buf,&mem[off],len); return true; }
  bool write(uint32_t off, const void* buf, size_t len) override {
    if(off+len>mem.size()) return false;
    writes++;
    const uint8_t* p=(const uint8_t*)buf;
    for(size_t i=0;i<len;i++){
      if(dieAfterBytes==0) return false;                       // 断电: 后面的字节没写进去
      if((p[i] & ~mem[off+i]) != 0) return false;              // 试图 0→1: NOR 不允许
      mem[off+i]=p[i]; if(dieAfterBytes>0) dieAfterBytes--;
    }
    return true;
  }
  bool eraseSector(uint32_t off) override { if(off%4096||off+4096>mem.size()) return false; erases++; memset(&mem[off],0xFF,4096); return true; }
};
static void fill(std::vector<int16_t>& d, int n, int base){ d.resize(3*n); for(int i=0;i<n;i++){ d[3*i]=base+i; d[3*i+1]=-(base+i); d[3*i+2]=8192; } }

int main(){
  const int N=500; std::vector<int16_t> d, rd(3*672);
  // 1) 空日志: 追加 3 块(模拟离线), 状态/顺序/内容; 标已发后不再是未发
  {
    SimFlash f(16*4096); WaveLog L; check("begin 空分区", L.begin(&f,f.mem.size()) && L.sectors()==16);
    check("空日志全空白", L.countBlank()==16 && L.oldestUnsent()==-1);
    for(int k=0;k<3;k++){ fill(d,N,k*1000); check("append", L.append(1787466254487LL,k,1787466239487LL+k*5000,N,100,8192,d.data())==k); }
    check("3 未发, 事件中无擦除(扇区本就空白)", L.countUnsent()==3 && L.inEventErases()==0 && f.erases==0);
    WaveLogHeader h; int s=L.oldestUnsent(); check("最旧未发=扇区0", s==0);
    check("readRecord crc 通过", L.readRecord(s,h,rd.data(),rd.size()*2));
    check("头字段正确", h.ev_ms==1787466254487LL && h.chunk_seq==0 && h.t0_ms==1787466239487LL && h.n==N && h.fs==100 && h.lsb_g==8192);
    check("数据正确", rd[0]==0 && rd[3*499]==499 && rd[1]==0 && rd[2]==8192);
    check("markSent 只写 4 字节且不擦", L.markSent(s) && f.erases==0);
    check("标记后最旧未发=扇区1", L.oldestUnsent()==1 && L.countSent()==1);
  }
  // 2) 断电恢复: 重新 begin 同一块 flash -> 未发记录仍在, 已发仍已发, 写指针接在最后一条之后, rid 单调
  {
    SimFlash f(16*4096); WaveLog L; L.begin(&f,f.mem.size());
    for(int k=0;k<5;k++){ fill(d,N,k); L.append(1LL,k,1000LL*k,N,100,8192,d.data()); }
    L.markSent(0); L.markSent(1); L.markSent(2);
    WaveLog R; check("重启后 begin", R.begin(&f,f.mem.size()));
    check("重启后未发 2 条仍在, 已发 3 条被开机回收成空白", R.countUnsent()==2 && R.countSent()==0 && R.bootErased()==3 && R.countBlank()==14);
    check("重启后最旧未发=扇区3", R.oldestUnsent()==3);
    fill(d,N,99); int s=R.append(2LL,0,5000LL,N,100,8192,d.data());
    check("重启后写指针接在扇区5", s==5);
    WaveLogHeader h; R.readRecord(5,h,rd.data(),rd.size()*2); check("rid 单调(=6)", h.rid==6);
  }
  // 3) 写到一半断电: 数据写了一半就断 -> 该扇区判坏、不当成未发; 维护时被擦掉
  {
    SimFlash f(16*4096); WaveLog L; L.begin(&f,f.mem.size());
    fill(d,N,1); L.append(1LL,0,0LL,N,100,8192,d.data());
    f.dieAfterBytes=1000; fill(d,N,2); check("半写 append 返回失败", L.append(1LL,1,5000LL,N,100,8192,d.data())<0);
    f.dieAfterBytes=-1;
    WaveLog R; R.begin(&f,f.mem.size());
    check("半写扇区不算未发(只有 1 条未发)", R.countUnsent()==1 && R.oldestUnsent()==0);
    // 另一种: 数据写完, 头写了一半断电(magic 在, crc 字段残缺)
    SimFlash g(16*4096); WaveLog M; M.begin(&g,g.mem.size());
    g.dieAfterBytes=N*6+20; fill(d,N,3); M.append(1LL,0,0LL,N,100,8192,d.data()); g.dieAfterBytes=-1;
    WaveLog R2; R2.begin(&g,g.mem.size());
    check("头半写(crc 残缺)判坏, 不算未发", R2.countUnsent()==0);
    check("开机把坏扇区擦成空白", R2.bootErased()==1 && R2.countBlank()==16);
    // 首次启用: 分区里是旧分区表残留的任意数据 -> 全判坏, 开机全擦, 之后事件中零就地擦除
    SimFlash j(16*4096); for(size_t i=0;i<j.mem.size();i++) j.mem[i]=(uint8_t)(i*7+3);
    WaveLog F; F.begin(&j,j.mem.size());
    check("残留数据全部在开机回收", F.bootErased()==16 && F.countBlank()==16);
    for(int k=0;k<16;k++){ fill(d,N,k); F.append(3LL,k,0LL,N,100,8192,d.data()); }
    check("回收后写满零就地擦除", F.inEventErases()==0 && F.countUnsent()==16);
  }
  // 4) 维护与擦除策略: 发完后 maintain 每次只擦 maxErase 个; 空闲擦干净后下一事件 append 零擦除
  {
    SimFlash f(16*4096); WaveLog L; L.begin(&f,f.mem.size());
    for(int k=0;k<10;k++){ fill(d,N,k); L.append(1LL,k,0LL,N,100,8192,d.data()); L.markSent(k); }
    check("maintain(1) 只擦 1 个", L.maintain(1)==1 && L.countBlank()==7);
    check("maintain(100) 擦完剩余 9 个", L.maintain(100)==9 && L.countBlank()==16);
    for(int k=0;k<16;k++){ fill(d,N,k); L.append(2LL,k,0LL,N,100,8192,d.data()); }
    check("擦干净后写满 16 块零就地擦除", L.inEventErases()==0);
  }
  // 5) 环绕: 离线积压超过容量 -> 覆盖最旧的未发(就地擦), 新的保住; 最旧未发按 rid 正确
  {
    SimFlash f(8*4096); WaveLog L; L.begin(&f,f.mem.size());
    for(int k=0;k<11;k++){ fill(d,N,k); L.append(1LL,k,100LL*k,N,100,8192,d.data()); }
    check("写满环绕后仍 8 条未发", L.countUnsent()==8);
    check("环绕覆盖 3 次=3 次就地擦", L.inEventErases()==3);
    WaveLogHeader h; L.readRecord(L.oldestUnsent(),h,rd.data(),rd.size()*2);
    check("最旧未发是 chunk 3(0~2 被覆盖)", h.chunk_seq==3);
    // 重启后同样结论
    WaveLog R; R.begin(&f,f.mem.size()); R.readRecord(R.oldestUnsent(),h,rd.data(),rd.size()*2);
    check("重启后最旧未发仍是 chunk 3", h.chunk_seq==3 && R.countUnsent()==8);
  }
  // 6) 补传流程(固件用法): 离线写 6 块 -> 联网逐条 oldestUnsent/read/POST(模拟前 2 次失败)/markSent -> 清空
  {
    SimFlash f(16*4096); WaveLog L; L.begin(&f,f.mem.size());
    for(int k=0;k<6;k++){ fill(d,N,k); L.append(7LL,k,0LL,N,100,8192,d.data()); }
    int posts=0, failLeft=2; std::vector<int> order;
    while(L.oldestUnsent()>=0 && posts<100){
      int s=L.oldestUnsent(); WaveLogHeader h; if(!L.readRecord(s,h,rd.data(),rd.size()*2)) break;
      posts++; if(failLeft>0){ failLeft--; continue; }      // POST 失败: 不标记, 下次重试同一条
      order.push_back((int)h.chunk_seq); L.markSent(s);
    }
    bool inOrder=true; for(int i=0;i<(int)order.size();i++) if(order[i]!=i) inOrder=false;
    check("补传按块序号顺序且失败会重试", inOrder && order.size()==6 && posts==8);
    check("补传完无未发", L.countUnsent()==0);
  }
  // 7) crc32 与 zlib 一致(已知向量 "123456789" -> 0xCBF43926)
  check("crc32 标准向量", WaveLog::crc32("123456789",9)==0xCBF43926u);
  std::printf(fails?"\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
