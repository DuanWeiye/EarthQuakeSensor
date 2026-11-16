#pragma once
// 流式計測震度(constant-memory, 事件不限长)。
//
// 思路: 計測震度的 a0 = "滤波后矢量幅度第 round(0.3*fs) 大的样本"(100Hz -> 第30大)。
// "取第K大"天然可流式: 维护一个 K 元素最小堆, 边喂边更新, 堆顶即 a0。
// 唯一要流式化的是 JMA 滤波 G(f)=FF*FH*FL —— 把已验证的 jma_filter_gain() 做逆FFT
// 得到时域脉冲响应, 截断成对称 FIR, 逐样本卷积。整段事件内存恒定, 不再受 2048 样本窗限制,
// 且用的是"整段最强 0.3s"而非"最后20s", 修正长事件低估。
//
// 用法: reset() -> 每样本 push(dx,dy,dz)(去重力后的动态加速度, gal) -> finish() -> keisoku()。
// 检测器用法: 空闲期每样本 feed() 只写延迟线(不算 FIR); 触发时 begin_event() 只清 top-K、保留延迟线,
// 这样事件开头的滤波输出用的是真实的触发前波形(而非补零), 震度不受触发时刻早晚影响。
// 纯 C++17, 不依赖 Arduino, 可主机端用 FFT 版对拍。
class JmaStream {
 public:
  static const int MAX_M = 512;           // FIR 半窗上限 -> 最长 L=1025 抽头(~10s 脉冲支撑)
  // fs: 采样率; half_m: FIR 半窗(抽头数 = 2*half_m+1, 越大越贴近频域 oracle)。
  explicit JmaStream(double fs, int half_m = 256);
  void reset();                            // 开始新事件: 清延迟线 + top-K
  void begin_event();                      // 开始新事件: 只清 top-K, 保留延迟线里的触发前波形
  void feed(double dx, double dy, double dz);  // 空闲期: 只写延迟线, 不出滤波值(3 次赋值, 开销可忽略)
  void push(double dx, double dy, double dz);  // 事件期: 写延迟线并出一个滤波值入 top-K
  void finish();                           // 事件结束: 冲刷 FIR 尾部(补 M 个零样本; 延迟线随后由 feed 重新填满)
  double a0() const;                        // 第K大滤波幅度(gal); 不足K个样本返回0
  double keisoku() const;                   // 計測震度 = round(2*log10(a0)+0.94)
 private:
  double fs_;
  int M_, L_, K_;
  double cf_[2*MAX_M+1];                    // 对称 FIR 抽头(展开, cf_[M]为中心)
  float  dx_[2*MAX_M+1], dy_[2*MAX_M+1], dz_[2*MAX_M+1];  // 三轴环形延迟线
  int wpos_;
  bool built_ = false;                      // 抽头是否已构建(惰性: 避免全局静态构造期堆分配崩溃)
  double heap_[64];                         // top-K 最小堆(K=round(0.3*fs), 100Hz->30)
  int heapN_;
  void buildTaps();
  void emit();                             // 对当前延迟线算一次滤波输出 -> 喂堆
  void heapPush(double v);
};
