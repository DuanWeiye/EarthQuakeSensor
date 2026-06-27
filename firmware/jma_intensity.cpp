#include "jma_intensity.h"
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <functional>

double jma_filter_gain(double f) {
  if (f <= 0.0) return 0.0;                 // DC：去掉（含重力）
  const double FF = std::sqrt(1.0 / f);     // 周期効果フィルタ
  const double y  = f / 10.0;               // ハイカット
  const double y2 = y*y, y4=y2*y2, y6=y4*y2, y8=y4*y4, y10=y8*y2, y12=y6*y6;
  const double FH = std::pow(1.0 + 0.694*y2 + 0.241*y4 + 0.0557*y6
                             + 0.009664*y8 + 0.00134*y10 + 0.000155*y12, -0.5);
  const double FL = std::sqrt(1.0 - std::exp(-std::pow(f/0.5, 3.0)));  // ローカット
  return FF * FH * FL;
}

void fft_radix2(double* re, double* im, int n, bool inverse) {
  // 位反转置换
  for (int i=1, j=0; i<n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { std::swap(re[i], re[j]); std::swap(im[i], im[j]); }
  }
  for (int len=2; len<=n; len<<=1) {
    double ang = (inverse ? 2.0 : -2.0) * M_PI / len;
    double wr = std::cos(ang), wi = std::sin(ang);
    for (int i=0; i<n; i+=len) {
      double cwr=1.0, cwi=0.0;
      for (int k=0; k<len/2; k++) {
        double ur=re[i+k],        ui=im[i+k];
        double vr=re[i+k+len/2]*cwr - im[i+k+len/2]*cwi;
        double vi=re[i+k+len/2]*cwi + im[i+k+len/2]*cwr;
        re[i+k]=ur+vr;        im[i+k]=ui+vi;
        re[i+k+len/2]=ur-vr;  im[i+k+len/2]=ui-vi;
        double ncwr=cwr*wr-cwi*wi; cwi=cwr*wi+cwi*wr; cwr=ncwr;
      }
    }
  }
  if (inverse) for (int i=0;i<n;i++){ re[i]/=n; im[i]/=n; }
}

double round_keisoku(double I) {
  double r2 = std::round(I * 100.0) / 100.0;   // 四舍五入到 2 位
  return std::floor(r2 * 10.0) / 10.0;          // 截断到 1 位
}

const char* shindo_scale(double k) {
  if (k < 0.5) return "0";
  if (k < 1.5) return "1";
  if (k < 2.5) return "2";
  if (k < 3.5) return "3";
  if (k < 4.5) return "4";
  if (k < 5.0) return "5-";
  if (k < 5.5) return "5+";
  if (k < 6.0) return "6-";
  if (k < 6.5) return "6+";
  return "7";
}

static void filter_one_axis(const float* a, int n, double fs, std::vector<double>& out) {
  std::vector<double> re(n), im(n, 0.0);
  for (int i=0;i<n;i++) re[i]=a[i];
  fft_radix2(re.data(), im.data(), n, false);
  for (int k=0;k<n;k++) {
    int kk = (k <= n/2) ? k : (n - k);            // 实信号谱对称, 用对应正频率
    double g = jma_filter_gain((double)kk * fs / n);
    re[k]*=g; im[k]*=g;
  }
  fft_radix2(re.data(), im.data(), n, true);
  out.resize(n);
  for (int i=0;i<n;i++) out[i]=re[i];             // 取实部为滤波后时域
}

double compute_keisoku_shindo(const float* ax, const float* ay, const float* az,
                              int n, double fs) {
  std::vector<double> fx, fy, fz;
  filter_one_axis(ax, n, fs, fx);
  filter_one_axis(ay, n, fs, fy);
  filter_one_axis(az, n, fs, fz);
  std::vector<double> mag(n);
  for (int i=0;i<n;i++) mag[i]=std::sqrt(fx[i]*fx[i]+fy[i]*fy[i]+fz[i]*fz[i]);
  int idx = (int)std::lround(0.3 * fs);            // 0.3s 对应样本数(100Hz->30)
  if (idx < 1) idx = 1;
  if (idx > n) idx = n;
  std::nth_element(mag.begin(), mag.begin()+(idx-1), mag.end(), std::greater<double>());
  double a = mag[idx-1];                            // 第 idx 大 = 累计 0.3s 的阈值
  if (a <= 0.0) return 0.0;
  return round_keisoku(2.0*std::log10(a) + 0.94);
}
