#pragma once
#include <cstdint>

// 原始加速度环形缓冲(单生产者/单消费者, 无锁)。
//
// 采样核每个样本 push 一组三轴 int16 原始 ADC 读数; 网络核按"绝对样本序号"读取连续段,
// 在事件进行中就分块上报(触发前 PRE 秒也在环里, P 波不丢), 事件长度不受缓冲容量限制:
// 环只需吸收网络抖动(容量 - 预触发 = 可容忍的断传时长)。
//
// 绝对序号与 Detector::sample_idx() 同步: 固件对每个样本先 push 到环、再 push 到检测器,
// 检测器给出的事件起止序号可直接索引本环。序号用 uint32(100Hz 下 497 天回绕), 比较一律用差值。
//
// 并发: head_ 用 acquire/release 原子读写; 消费者拷贝完再核对区段未被覆盖(类 seqlock),
// 被覆盖即返回 -1 让调用方跳到 oldest()。纯 C++17, 不依赖 Arduino, 可主机单测。
class WaveRing {
 public:
  bool begin(int capSamples);                 // setup 里调用(堆就绪后)分配; 失败返回 false
  int  capacity() const { return cap_; }
  void push(int16_t x, int16_t y, int16_t z); // 生产者; 满则覆盖最旧
  uint32_t head() const;                      // 已写入样本数 = 下一样本的绝对序号
  uint32_t oldest() const;                    // 仍可读的最小序号
  int available(uint32_t from) const;         // from 起到 head 之间有多少样本(from 超前返回 0)
  // 消费者: 从绝对序号 from 起拷贝最多 n 个样本(交错 x,y,z, 共 3n 个 int16)到 out。
  // 返回实际拷贝数(0=尚无新样本); from 已被覆盖(或拷贝中被覆盖)返回 -1。
  int read(uint32_t from, int n, int16_t* out) const;
 private:
  int16_t* buf_ = nullptr;
  int cap_ = 0;
  uint32_t head_ = 0;
};
