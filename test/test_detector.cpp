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
    // 时间基准: start_s/end_s 与 now_s 同一采样计数基准, 固件据此把事件起止折算成 epoch
    check("end_s-start_s ≈ dur", got && std::fabs((ev.end_s-ev.start_s)-ev.dur_s)<0.02);
    check("now_s 在事件结束之后(含 hold-off)", got && d.now_s()>ev.end_s && d.now_s()-ev.end_s<3.0);
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

  // 7) 自适应曲线: 50s 长震 -> 包络格宽自动翻倍, 仍全程覆盖、点数不超上限
  {
    Detector d=mk(fs);
    quiet(d,200);
    for(int i=0;i<5000;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs); d.push(s,0,980.665f); } // 50s
    quiet(d,200);
    EventSummary ev; bool got=d.poll_event(ev);
    check("长事件入库", got);
    check("长事件 dur≈50s", got && ev.dur_s>45.0);
    check("格宽已翻倍(>100ms)", got && ev.env_dt_ms>100.0);
    check("点数不超上限", got && ev.env_n>0 && ev.env_n<=EQ_ENV_CAP);
    double cover = got ? ev.env_n*ev.env_dt_ms/1000.0 : 0.0;   // 曲线覆盖时长(含 hold-off 拖尾)
    check("曲线全程覆盖(无截断)", got && cover > ev.dur_s && cover < ev.dur_s+3.0);
    check("长事件 keisoku>0(不限长)", got && ev.keisoku>0.0);
  }


  // ---- 以下为 2026-08-23 茨城 M5.9 实震暴露问题的回归(起停迟滞 + 抗污染噪声底) ----
  // 实震配置(与服务端默认一致): floor 10 / K 6 / hold 1.5 / off: K2 floor3
  auto rcfg=[](){ DetConfig c; c.k=6.0; c.floor_min=10.0; c.holdoff_s=1.5; c.min_dur_s=2.0;
                  c.led_min_s=0.5; c.min_keisoku=0.0; c.k_off=2.0; c.floor_off=3.0; c.tail_max_s=120.0; return c; };
  // 确定性伪随机噪声(±amp 均匀), 模拟住宅安装点底噪
  uint32_t rng=12345u; auto noise=[&](float amp){ rng=rng*1664525u+1013904223u; return amp*(((rng>>8)&0xFFFF)/32768.0f-1.0f); };

  // 8) 实震形态: 60s 底噪 -> 8s P波前驱(3→8gal, 低于触发阈值但远高于底噪) -> 10s S波 60gal
  //    -> 25s 尾波 30gal 指数衰减到 4gal -> 静。旧逻辑: 前驱把 EMA 噪声底抬到 ~5gal, 阈值 30+,
  //    尾波在 S 波一停就被切掉(实震只记到 10.0s)。新逻辑必须: 噪声底不被前驱污染, 尾波全程记录。
  {
    Detector d(fs, rcfg());
    for(int i=0;i<6000;i++) d.push(noise(1.5f),noise(1.5f),980.665f+noise(1.5f));
    double nfQuiet=d.noise_floor();
    check("底噪下噪声底已学到(0.1~2gal)", nfQuiet>0.1 && nfQuiet<2.0);
    for(int i=0;i<800;i++){ float A=3.0f+5.0f*i/800.0f; float s=A*std::sin(2*M_PI*5.0*i/fs);
      d.push(s+noise(1.5f),noise(1.5f),980.665f+noise(1.5f)); }
    check("P波前驱(<floor_min)不触发", !d.active());
    check("P波前驱不污染噪声底(分位估计)", d.noise_floor() < nfQuiet*1.5+0.2);
    int nStrong=1000, nCoda=2500; bool ledDuringCoda=false, activeDuringCoda=true;
    for(int i=0;i<nStrong;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs); d.push(s,0,980.665f); }
    double tau=25.0/std::log(30.0/4.0);
    for(int i=0;i<nCoda;i++){ float A=30.0f*std::exp(-(i/fs)/tau); float s=A*std::sin(2*M_PI*2.0*i/fs);
      d.push(s,0,980.665f);
      if(i>1500){ if(d.led_on()) ledDuringCoda=true; if(!d.active()) activeDuringCoda=false; } }
    check("尾波(>3gal 但 <触发阈值)期间事件仍在进行", activeDuringCoda);
    check("尾波弱晃期间 LED 已灭(LED 跟强晃)", !ledDuringCoda);
    for(int i=0;i<1000;i++) d.push(noise(1.5f),noise(1.5f),980.665f+noise(1.5f));
    EventSummary ev; bool got=d.poll_event(ev);
    check("实震形态入库", got);
    check("全程时长≈35s(尾波全留住, 非旧版的 10s)", got && ev.dur_s>30.0 && ev.dur_s<36.5);
    check("强晃时长≈S波10s+尾波>10gal段", got && ev.strong_s>20.0 && ev.strong_s<26.0);
    check("dur_s ≥ strong_s", got && ev.dur_s>=ev.strong_s);
    double cover = got ? ev.env_n*ev.env_dt_ms/1000.0 : 0.0;
    check("包络覆盖全程(含尾波)", got && cover>=ev.dur_s && cover<ev.dur_s+3.0);
    check("事件结束后噪声底恢复学习且仍在底噪量级", d.noise_floor()<2.0);
  }

  // 9) 录入门限按"强晃"判: 0.5s 强敲 + 5s 弱晃(超收尾阈值但低于触发阈值) -> 不入库(轻碰+余振)
  {
    Detector d(fs, rcfg());
    for(int i=0;i<500;i++) d.push(noise(1.0f),noise(1.0f),980.665f+noise(1.0f));
    for(int i=0;i<50;i++){ float s=80.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f); }
    for(int i=0;i<500;i++){ float s=5.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f); }
    quiet(d,300);
    EventSummary ev;
    check("强晃不足 min_dur 的轻碰+余振不入库", !d.poll_event(ev));
  }

  // 10) tail_max 兜底: 3s 强晃后旁边一直有 5gal 弱扰动(永不低于收尾阈值) -> 事件仍须在 tail_max 内结束
  {
    DetConfig c=rcfg(); c.tail_max_s=5.0;
    Detector d(fs, c);
    for(int i=0;i<500;i++) d.push(noise(1.0f),noise(1.0f),980.665f+noise(1.0f));
    for(int i=0;i<300;i++){ float s=60.0f*std::sin(2*M_PI*2.0*i/fs); d.push(s,0,980.665f); }
    int endedAt=-1;
    for(int i=0;i<3000;i++){ float s=5.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f);
      if(!d.active() && endedAt<0) endedAt=i; }
    EventSummary ev; bool got=d.poll_event(ev);
    check("持续弱扰动下事件在 tail_max 内强制收尾", endedAt>0 && endedAt<=(int)(5.0*fs)+5);
    check("强制收尾的事件仍入库", got && ev.strong_s>2.5 && ev.strong_s<3.5);
  }

  // 11) 噪声底分位估计: 100s 安静(1gal 振幅) 后来 20s 8gal 持续扰动(不触发) -> 噪声底仍≈安静值
  {
    Detector d(fs, rcfg());
    for(int i=0;i<10000;i++){ float s=1.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f); }
    double nf0=d.noise_floor();
    for(int i=0;i<2000;i++){ float s=8.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f); }
    check("20s 扰动未触发事件", !d.active());
    check("噪声底不随 20s 扰动飙升(旧 EMA 会到 ~5gal)", d.noise_floor() < nf0*1.3+0.05);
    for(int i=0;i<10000;i++){ float s=8.0f*std::sin(2*M_PI*5.0*i/fs); d.push(s,0,980.665f); }
    check("扰动持续 >窗口 后噪声底跟上新环境(仍自适应)", d.noise_floor() > 3.0);
  }

  std::printf(fails?"\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
