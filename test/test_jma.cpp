// 主机端单测：用 g++ 编译运行。期望值由官方公式 FF*FH*FL 手算(见计划)。
#include "../firmware/jma_intensity.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <string>

static int fails = 0;
static void approx(const char* name, double got, double exp, double tol) {
  if (std::fabs(got - exp) > tol) { std::printf("FAIL %s: got %.6f exp %.6f\n", name, got, exp); fails++; }
  else std::printf("ok   %s: %.6f\n", name, got);
}

static void test_fft() {
  // 正弦 -> FFT 在对应 bin 出峰；逆变换可还原。
  const int N = 64;
  double re[64], im[64], re0[64];
  for (int k=0;k<N;k++){ re[k]=std::cos(2*M_PI*5*k/N); im[k]=0; re0[k]=re[k]; }
  fft_radix2(re, im, N, false);
  double mag5 = std::hypot(re[5], im[5]);          // bin 5 应有能量(~N/2)
  approx("fft bin5 mag", mag5, N/2.0, 1.0);
  fft_radix2(re, im, N, true);                     // 逆变换还原
  double err=0; for(int k=0;k<N;k++) err += std::fabs(re[k]-re0[k]);
  approx("fft roundtrip err", err, 0.0, 1e-6);
}

static void test_round_scale() {
  approx("round 3.456", round_keisoku(3.456), 3.4, 1e-9);  // ->3.46->3.4
  approx("round 4.999", round_keisoku(4.999), 5.0, 1e-9);  // ->5.00->5.0
  // 阶级边界
  #define SCALE_EQ(I, S) do{ const char* s=shindo_scale(I); \
    if (std::string(s)!=S){ std::printf("FAIL scale %.2f: got %s exp %s\n", I, s, S); fails++; } \
    else std::printf("ok   scale %.2f -> %s\n", I, s); }while(0)
  SCALE_EQ(0.49, "0"); SCALE_EQ(0.5, "1"); SCALE_EQ(2.5, "3");
  SCALE_EQ(4.5, "5-"); SCALE_EQ(5.0, "5+"); SCALE_EQ(5.9, "6-");
  SCALE_EQ(6.0, "6+"); SCALE_EQ(6.5, "7");
}

static void test_keisoku() {
  // 在 X 轴放 bin 对齐的正弦, 幅度 A(gal); 滤波后幅度≈A*G(f), 0.3s 值≈该幅度。
  const int N = 2048; const double fs = 100.0;
  const int kbin = 20; const double f = kbin * fs / N;     // ≈0.9766Hz, 整周期无泄漏
  const double A = 100.0;
  static float ax[2048], ay[2048], az[2048];
  for (int i=0;i<N;i++){ ax[i]=(float)(A*std::sin(2*M_PI*f*i/fs)); ay[i]=0; az[i]=0; }
  double got = compute_keisoku_shindo(ax, ay, az, N, fs);
  double a_exp = A * jma_filter_gain(f);
  double I_exp = round_keisoku(2*std::log10(a_exp) + 0.94);
  approx("keisoku sinusoid", got, I_exp, 0.15);  // 离散/排序误差容差
}

int main() {
  approx("G(0.5)", jma_filter_gain(0.5), 1.1234, 0.01);
  approx("G(1.0)", jma_filter_gain(1.0), 0.9964, 0.01);
  approx("G(10)",  jma_filter_gain(10.0), 0.2234, 0.01);
  approx("G(0)",   jma_filter_gain(0.0), 0.0, 1e-9);   // DC 去除
  test_fft();
  test_round_scale();
  test_keisoku();
  std::printf(fails ? "\n%d FAIL\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
