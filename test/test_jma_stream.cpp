// 流式計測震度(JmaStream)对拍测试: 以已验证的 FFT 版 compute_keisoku_shindo 为 oracle,
// 验证逐样本流式实现得出的震度一致; 再验证"长事件不限长 + 强震段在前不被丢"。
#include "../firmware/jma_stream.h"
#include "../firmware/jma_intensity.h"
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>

static const double FS = 100.0;
static int g_fail = 0;

static void check(const char* name, double got, double want, double tol){
  double d = std::fabs(got - want);
  bool ok = d <= tol;
  if(!ok) g_fail++;
  printf("  [%s] %-34s stream=%.3f oracle=%.3f |Δ|=%.3f (tol=%.2f)\n",
         ok?"PASS":"FAIL", name, got, want, d, tol);
}

// 往三轴(0/1/2)某轴叠加一段 Hann 窗正弦突发
static void addBurst(std::vector<double>& a, int axisLen, double* ax, double* ay, double* az,
                     int axis, double f, double amp, int center, int blen){
  double* arr = axis==0?ax : axis==1?ay : az;
  int s = center - blen/2;
  for(int n=0; n<blen; n++){
    int i = s + n; if(i<0 || i>=axisLen) continue;
    double w = 0.5*(1.0 - std::cos(2.0*M_PI*n/(blen-1)));   // Hann
    arr[i] += amp * w * std::sin(2.0*M_PI*f*(double)i/FS);
  }
}

// 用流式实现整段算一遍
static double streamKeisoku(const double* ax, const double* ay, const double* az, int n, int half_m){
  JmaStream js(FS, half_m);
  js.reset();
  for(int i=0;i<n;i++) js.push(ax[i], ay[i], az[i]);
  js.finish();
  return js.keisoku();
}

// 一个对拍用例: 在 N=4096 窗内中部放突发(两端为零, 圆周≈线性, oracle 公平)
struct Case { const char* name; double f; double amp; int axis; int blen; };

int main(){
  const int N = 4096;             // 40.96s 窗(oracle 需 2 的幂)
  const int HM = 256;             // FIR 半窗(2.56s 支撑); 与 oracle 误差应 <0.1
  printf("== JmaStream vs FFT-oracle 对拍 (fs=%.0f, FIR half=%d) ==\n", FS, HM);

  Case cases[] = {
    {"1.0Hz x amp50  burst8s",  1.0,  50, 0, 800},
    {"2.0Hz x amp80  burst6s",  2.0,  80, 0, 600},
    {"0.7Hz z amp120 burst10s", 0.7, 120, 2, 1000},
    {"3.0Hz y amp40  burst8s",  3.0,  40, 1, 800},
    {"5.0Hz x amp100 burst4s",  5.0, 100, 0, 400},
  };
  for(auto& c : cases){
    std::vector<double> ax(N,0.0), ay(N,0.0), az(N,0.0);
    addBurst(ax, N, ax.data(), ay.data(), az.data(), c.axis, c.f, c.amp, N/2, c.blen);
    double oracle = compute_keisoku_shindo(
        std::vector<float>(ax.begin(),ax.end()).data(),
        std::vector<float>(ay.begin(),ay.end()).data(),
        std::vector<float>(az.begin(),az.end()).data(), N, FS);
    double s = streamKeisoku(ax.data(), ay.data(), az.data(), N, HM);
    check(c.name, s, oracle, 0.1);
  }

  // 多频混合(更贴近真实地震波)
  {
    std::vector<double> ax(N,0.0), ay(N,0.0), az(N,0.0);
    addBurst(ax,N,ax.data(),ay.data(),az.data(), 0, 1.5, 60, N/2, 800);
    addBurst(ax,N,ax.data(),ay.data(),az.data(), 1, 3.0, 40, N/2, 800);
    addBurst(ax,N,ax.data(),ay.data(),az.data(), 2, 0.8, 50, N/2, 900);
    double oracle = compute_keisoku_shindo(
        std::vector<float>(ax.begin(),ax.end()).data(),
        std::vector<float>(ay.begin(),ay.end()).data(),
        std::vector<float>(az.begin(),az.end()).data(), N, FS);
    double s = streamKeisoku(ax.data(), ay.data(), az.data(), N, HM);
    check("混合 1.5/3.0/0.8Hz", s, oracle, 0.1);
  }

  // 不限长 + 强震段在前: 60s 信号, 强突发在 0~15s, 之后全静。
  // 旧"取最后2048样本"会只看到尾部静段 -> 严重低估; 流式应抓住前段强震。
  {
    const int LONG = 6000;                 // 60s
    std::vector<double> ax(LONG,0.0), ay(LONG,0.0), az(LONG,0.0);
    addBurst(ax,LONG,ax.data(),ay.data(),az.data(), 0, 1.2, 90, 750, 1400);  // 中心7.5s
    // 同样的突发单独算(短窗)的震度, 作为"应当抓到"的参考
    std::vector<double> bx(2048,0.0), by(2048,0.0), bz(2048,0.0);
    addBurst(bx,2048,bx.data(),by.data(),bz.data(), 0, 1.2, 90, 1024, 1400);
    double burstOnly = streamKeisoku(bx.data(), by.data(), bz.data(), 2048, HM);
    double full = streamKeisoku(ax.data(), ay.data(), az.data(), LONG, HM);
    printf("  [info] 强震段单独震度=%.3f  整段60s震度=%.3f\n", burstOnly, full);
    check("长事件强震在前(不丢)", full, burstOnly, 0.1);
    if(full < 3.5) { g_fail++; printf("  [FAIL] 整段震度过低, 疑似只看尾部静段\n"); }
  }

  printf(g_fail? "\n== %d 个用例失败 ==\n" : "\n== 全部通过 ==\n", g_fail);
  return g_fail? 1 : 0;
}
