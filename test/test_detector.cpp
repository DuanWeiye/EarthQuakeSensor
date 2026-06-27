#include "../firmware/detector.h"
#include <cstdio>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static int fails=0;
static void check(const char* n, bool c){ std::printf(c?"ok   %s\n":"FAIL %s\n", n); if(!c) fails++; }

static void quiet(Detector& d, int n){ for(int i=0;i<n;i++) d.push(0.0f,0.0f,980.665f); }

// 测试用配置: 低门槛便于触发; min_keisoku=0 不拦截(单独测门限)
static DetConfig tcfg(){
  DetConfig c; c.k=5.0; c.floor_min=8.0; c.holdoff_s=1.0;
  c.min_dur_s=2.0; c.led_min_s=0.5; c.min_keisoku=0.0; return c;
}
static Detector mk(double fs){ return Detector(fs, tcfg()); }

int main(){
  const double fs=100.0;

  // 1) 长震(3s): 记录 + 带包络; 持续晃约0.5s后 led_on 置位
  {
    Detector d=mk(fs);
    check("quiet not active", !d.push(0.0f,0.0f,980.665f));
    quiet(d,200);
    EventSummary ev; check("no event in quiet", !d.poll_event(ev));
    bool sawActive=false, sawLed=false;
    for(int i=0;i<300;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs);
      if(d.push(s,0,980.665f)) sawActive=true;
      if(d.led_on()) sawLed=true; }
    check("active during shake", sawActive);
    check("led_on 在持续晃动后点亮", sawLed);
    quiet(d,200);
    bool got=d.poll_event(ev);
    check("long event emitted", got);
    check("dur ~3s", got && ev.dur_s>2.5 && ev.dur_s<3.5);
    check("peak>30", got && ev.peak_gal>30.0);
    check("horizontal dominant", got && ev.peak_h_gal>ev.peak_v_gal);
    check("keisoku>0", got && ev.keisoku>0.0);
    check("not active after end", !d.active());
    check("led_off after end", !d.led_on());
    check("env_n>0", got && ev.env_n>0);
    bool envHas=false; for(int i=0;i<ev.env_n;i++) if(ev.env_tot[i]>30.0f) envHas=true;
    check("env_tot 有震动", got && envHas);
    bool envH=false; for(int i=0;i<ev.env_n;i++) if(ev.env_h[i]>ev.env_v[i]) envH=true;
    check("env 水平分量存在", got && envH);
  }

  // 2) 极短敲击(0.2s): 跨度 < LED 门限 -> 不亮灯, 不入库
  {
    Detector d=mk(fs);
    quiet(d,100);
    bool sawLed=false;
    for(int i=0;i<20;i++){ float s=80.0f*std::sin(2*M_PI*5.0*i/fs);
      d.push(s,0.0f,980.665f); if(d.led_on()) sawLed=true; }
    quiet(d,200);
    EventSummary ev;
    check("极短敲击全程不亮灯", !sawLed);
    check("极短敲击不入库", !d.poll_event(ev));
  }

  // 3) 最小震度门限: 同样的长震, 把 min_keisoku 抬到不可能值 -> 不入库
  {
    DetConfig c=tcfg(); c.min_keisoku=99.0;
    Detector d(fs,c);
    quiet(d,200);
    for(int i=0;i<300;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs); d.push(s,0,980.665f); }
    quiet(d,200);
    EventSummary ev;
    check("震度不够->不入库", !d.poll_event(ev));
  }

  // 4) 运行期 set_config 生效: 提高 floor_min 后, 同样 60gal 晃动不再触发
  {
    Detector d=mk(fs);
    quiet(d,200);
    DetConfig c=tcfg(); c.floor_min=100.0; d.set_config(c);
    bool anyActive=false;
    for(int i=0;i<300;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs);
      if(d.push(s,0,980.665f)) anyActive=true; }
    check("set_config 提高门槛后 60gal 不触发", !anyActive);
  }

  // 5) 遥测: recent_peak 取走即清零
  {
    Detector d=mk(fs);
    quiet(d,30);
    for(int i=0;i<30;i++) d.push(40.0f,0.0f,980.665f);   // 制造动态
    float p=d.take_recent_peak();
    check("recent_peak>0", p>0.0f);
    float p2=d.take_recent_peak();                         // 立即再取(中间无样本)
    check("recent_peak 取走后清零", p2 < 1.0f);
  }

  // 6) 回归: 设备晃动中移位, 停后静止姿态不同 -> 事件仍须能结束
  {
    Detector d=mk(fs);
    quiet(d,200);
    for(int i=0;i<300;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs); d.push(s,0,980.665f); }
    for(int i=0;i<800;i++) d.push(60.0f,0.0f,978.0f);
    check("移位后事件能结束(不卡住一直闪)", !d.active());
    check("移位后 led 灭", !d.led_on());
  }

  std::printf(fails?"\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
