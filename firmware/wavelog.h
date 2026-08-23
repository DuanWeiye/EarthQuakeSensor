#pragma once
#include <cstdint>
#include <cstddef>

// 原始波形 flash 日志(断网补传 / 断电留存)。
//
// 上报块(WAVE_CHUNK 样本)一律先写进专用分区, 再由网络核按记录号顺序补传; 断网时块留在 flash,
// 联网后自动发完; 断电后开机扫描分区恢复未发记录。每块独占一个 4KB 扇区(头 64B + 数据 ≤4032B):
//   空白(全 0xFF) / 未发(sent==0xFFFFFFFF) / 已发(sent==0) / 坏(magic 或 crc 不符, 如写到一半断电)
// "已发"标记是把头部 sent 从 0xFFFFFFFF 改写成 0 —— NOR flash 的位只能 1→0, 无需擦除, 一次 4 字节写。
//
// 擦除策略: 擦扇区会让 IDF 关 cache 并停住另一核 ~45ms(最坏 400ms), 采样核跟着停(vTaskDelayUntil 事后
// 补跑, 样本数不丢、只是挤在一起)。所以擦除放在空闲维护 maintain() 里零散做(每次一个), 事件中只做页写
// (~1ms); 只有日志里没有空白扇区了(已发的还没来得及擦, 或离线积压超过容量)才就地擦一次。
// 写满后环绕覆盖最旧的记录(含未发的: 离线超过日志容量只能丢最旧)。
// 纯 C++17, flash 操作经 FlashIO 抽象, 主机端用内存 NOR 模拟器测试(含断电/半写场景)。
struct FlashIO {
  virtual bool read(uint32_t off, void* buf, size_t len) = 0;
  virtual bool write(uint32_t off, const void* buf, size_t len) = 0;   // NOR 语义: 位只能 1→0
  virtual bool eraseSector(uint32_t off) = 0;                          // 4KB 对齐
  virtual ~FlashIO() {}
};

struct WaveLogHeader {    // 64 B, 小端, 位于扇区开头。字段按对齐排布, 无隐式填充(static_assert 保证)
  uint32_t magic;         // WAVELOG_MAGIC                                  @0
  uint32_t rid;           // 单调记录号: 断电后据此定位写指针、决定补传顺序  @4
  int64_t  ev_ms;         // 事件起点 epoch ms(与服务端 ev 同值)             @8
  int64_t  t0_ms;         // 本块首样本 epoch ms                             @16
  uint32_t chunk_seq;     // 块序号(事件内)                                  @24
  uint16_t n, fs, lsb_g, reserved;                                      // @28..35
  uint32_t crc;           // 数据(3n 个 int16)的 CRC32                       @36
  uint32_t sent;          // 0xFFFFFFFF 未发 / 0 已发                        @40
  uint8_t  pad[64 - 44];
};
static_assert(sizeof(WaveLogHeader) == 64, "WaveLogHeader 必须正好 64 字节");
static_assert(offsetof(WaveLogHeader, sent) == 40, "sent 偏移须与 WaveLog::SENT_OFF 一致");
static const uint32_t WAVELOG_MAGIC = 0x31474C57u;   // 'W','L','G','1' 小端

class WaveLog {
 public:
  static const int SECTOR = 4096, HDR = 64, MAX_SECTORS = 256;
  static const int MAX_DATA = SECTOR - HDR;           // 4032 B = 672 样本
  static const int SENT_OFF = 40;                     // sent 字段在头内偏移(4 字节对齐)
  bool begin(FlashIO* io, uint32_t sizeBytes);        // 扫描分区建状态表、定位写指针, 并擦掉已发/坏扇区; 不可用返回 false
  bool ok() const { return io_ != nullptr && nsec_ > 0; }
  int  sectors() const { return nsec_; }
  // 追加一块(事件中由网络核调用): 写到写指针所指扇区(非空白则就地擦), 返回扇区号或 -1
  int  append(int64_t ev_ms, uint32_t chunk_seq, int64_t t0_ms, uint16_t n, uint16_t fs, uint16_t lsb_g,
              const int16_t* data);
  int  oldestUnsent() const;                          // 记录号最小的未发扇区, 无则 -1
  bool readRecord(int sector, WaveLogHeader& h, int16_t* data, size_t dataCapBytes) const;   // 校验 crc
  bool markSent(int sector);
  int  maintain(int maxErase);                        // 空闲维护: 擦最多 maxErase 个已发/坏扇区, 返回擦了几个
  int  countUnsent() const; int countBlank() const; int countSent() const;
  int  inEventErases() const { return inEventErases_; }   // 统计: append 时被迫就地擦除的次数
  int  bootErased() const { return bootErased_; }         // 统计: begin 时回收(擦除)的已发/坏扇区数
  static uint32_t crc32(const void* data, size_t len);
 private:
  enum State : uint8_t { BLANK=0, UNSENT=1, SENT=2, BAD=3 };
  FlashIO* io_ = nullptr; int nsec_ = 0; int wp_ = 0; uint32_t nextRid_ = 1; int inEventErases_ = 0; int bootErased_ = 0;
  uint8_t  state_[MAX_SECTORS]; uint32_t rid_[MAX_SECTORS];
  void scanSector(int s);
};
