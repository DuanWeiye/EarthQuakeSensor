#pragma once
// 官方 JMA 计测震度滤波增益 G(f)=FF*FH*FL。f 单位 Hz；f<=0 返回 0（去 DC）。
double jma_filter_gain(double f);

// 原地 radix-2 FFT。n 为 2 的幂。inverse=true 做逆变换并除以 n。
void fft_radix2(double* re, double* im, int n, bool inverse);

double round_keisoku(double I);          // 小数第3位四舍五入, 第2位切舍 -> 1 位小数
const char* shindo_scale(double keisoku);// 计测震度 -> 震度阶级字符串

// 端到端计测震度：三轴加速度(gal)、样本数 n(2 的幂)、采样率 fs。
// 各轴 FFT→乘 G(f)→逆 FFT→三轴矢量合成→第 round(0.3*fs) 大样本=a→round_keisoku(2log10(a)+0.94)。
double compute_keisoku_shindo(const float* ax, const float* ay, const float* az,
                              int n, double fs);
