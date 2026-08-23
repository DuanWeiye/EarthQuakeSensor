#include "../firmware/wavering.h"
#include <cstdio>
#include <vector>
static int fails=0;
static void check(const char* n, bool c){ std::printf(c?"ok   %s\n":"FAIL %s\n", n); if(!c) fails++; }

int main(){
  // 1) 基本: 容量 1000(=10s@100Hz), 写 250 个, 整段读回且顺序/内容正确
  {
    WaveRing r; check("begin 分配成功", r.begin(1000));
    for(int i=0;i<250;i++) r.push((int16_t)i,(int16_t)(i*2),(int16_t)(-i));
    check("head=250", r.head()==250);
    check("oldest=0(未回绕)", r.oldest()==0);
    std::vector<int16_t> out(3*250);
    int got=r.read(0,250,out.data());
    bool ok=got==250; for(int i=0;i<250&&ok;i++) ok = out[3*i]==i && out[3*i+1]==i*2 && out[3*i+2]==-i;
    check("read 整段内容正确", ok);
    check("read 超前返回 0", r.read(250,100,out.data())==0);
    check("available 正确", r.available(100)==150 && r.available(250)==0 && r.available(300)==0);
  }
  // 2) 回绕覆盖: 容量 1000, 写 2500 -> 只剩最近 1000; 读已覆盖的序号返回 -1; 从 oldest 读正确
  {
    WaveRing r; r.begin(1000);
    for(int i=0;i<2500;i++) r.push((int16_t)i,0,0);
    check("回绕后 oldest=1500", r.oldest()==1500);
    std::vector<int16_t> out(3*500);
    check("读被覆盖区段返回 -1", r.read(1400,100,out.data())==-1);
    int got=r.read(1500,500,out.data());
    bool ok=got==500; for(int i=0;i<500&&ok;i++) ok = out[3*i]==(int16_t)(1500+i);
    check("从 oldest 读 500 个内容正确(跨物理环边界)", ok);
    got=r.read(2400,500,out.data());
    check("尾部不足 n 时只返回可用数", got==100 && out[0]==2400);
  }
  // 3) 流式消费模式(固件用法): 边写边按块读, 消费速度跟得上时无丢失
  {
    WaveRing r; r.begin(6000);              // 60s 环
    const int CHUNK=500; uint32_t next=0; long long sum=0; int chunks=0;
    std::vector<int16_t> out(3*CHUNK);
    for(int i=0;i<20000;i++){               // 模拟 200s 采样
      r.push((int16_t)(i%30000),0,0);
      if(r.available(next)>=CHUNK){ int g=r.read(next,CHUNK,out.data()); if(g>0){ for(int k=0;k<g;k++) sum+=out[3*k]; next+=g; chunks++; } }
    }
    long long expect=0; for(int i=0;i<20000;i++) expect+=i%30000;
    check("流式按块读 40 块", chunks==40);
    check("无丢失(总和一致)", sum==expect);
  }
  // 4) 消费者长时间卡住(网络断 70s > 环 60s): 恢复后 read 返回 -1, 跳到 oldest 继续, 不崩溃
  {
    WaveRing r; r.begin(6000);
    for(int i=0;i<7000;i++) r.push((int16_t)i,0,0);
    std::vector<int16_t> out(3*500);
    uint32_t next=0;
    int g=r.read(next,500,out.data());
    check("断传超过环容量后 read=-1", g==-1);
    next=r.oldest();
    g=r.read(next,500,out.data());
    check("跳到 oldest 后可继续读", g==500 && out[0]==(int16_t)1000);
  }
  // 5) uint32 序号回绕附近(差值比较)正确 —— 让 head 接近 2^32 不现实, 只验证 available 的差值语义
  {
    WaveRing r; r.begin(10);
    for(int i=0;i<15;i++) r.push(0,0,0);
    check("from 落在(oldest,head)内 available 正确", r.available(7)==8);
    check("from 超前 available=0", r.available(20)==0);
  }
  std::printf(fails?"\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
