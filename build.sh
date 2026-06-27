#!/bin/bash
# build.sh — earthQuakeSensor 固件 编译 & 烧录 (M5Stack Atom-Matrix / ESP32-PICO)
# 用法: ./build.sh (只编译) | ./build.sh -f (编译+烧录) | ./build.sh -w (只烧已有 bin)
# 目标设备写死, 不接受端口参数, 从根上防刷错。
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 机器专属配置(工具链路径 + 烧录目标)从本地 device.conf 读取, 不入库。
# 首次使用: cp device.conf.example device.conf 后按实际填写。
if [[ ! -f "$SCRIPT_DIR/device.conf" ]]; then
  echo "ERROR: 缺少 device.conf。请先 cp device.conf.example device.conf 并按你的机器/设备填写。"; exit 1
fi
# shellcheck source=/dev/null
source "$SCRIPT_DIR/device.conf"

SKETCH="$SCRIPT_DIR/firmware/firmware.ino"
TARGET="${EQ_FLASH_TARGET:-}"

FQBN="m5stack:esp32:m5stack_atom:\
FlashMode=dio,FlashFreq=80,FlashSize=4M,PartitionScheme=huge_app,\
CPUFreq=240,UploadSpeed=1500000,DebugLevel=none,LoopCore=1,EventsCore=1,EraseFlash=none"
BUILD_DIR="/tmp/eq_build"; OUT_DIR="$SCRIPT_DIR"; BIN_NAME="earthQuakeSensor.bin"
DO_BUILD=1; DO_FLASH=0
case "$1" in
  -f) DO_FLASH=1 ;;
  -w) DO_BUILD=0; DO_FLASH=1 ;;
  "" ) ;;
  *) echo "用法: $0 [-f|-w]"; exit 1 ;;
esac
mkdir -p "$BUILD_DIR"
if [[ $DO_BUILD -eq 1 ]]; then
  : "${ARDUINO:?device.conf 未设置 ARDUINO(Arduino 1.8.19 CLI 路径)}"
  echo "=== 编译 ==="; echo "  FQBN: $FQBN"
  "$ARDUINO" --board "$FQBN" --pref "build.path=$BUILD_DIR" --verify "$SKETCH"
  cp "$BUILD_DIR/firmware.ino.bin" "$OUT_DIR/$BIN_NAME"
  echo "=== 编译完成: $OUT_DIR/$BIN_NAME ($(stat -c%s "$OUT_DIR/$BIN_NAME") bytes) ==="
fi
if [[ $DO_FLASH -eq 1 ]]; then
  : "${ESPTOOL:?device.conf 未设置 ESPTOOL}"; : "${BOOT_APP:?device.conf 未设置 BOOT_APP}"
  # 安全闸：目标必须已配置且存在; 绝不回退到 ttyUSB0/ttyACM0 等别的板子
  if [[ -z "$TARGET" ]]; then echo "ERROR: device.conf 未设置 EQ_FLASH_TARGET(你的设备 by-id 路径)"; exit 1; fi
  if [[ ! -e "$TARGET" ]]; then echo "ERROR: 烧录目标不存在: $TARGET (设备没接? 或换口了?) 拒绝烧录"; exit 1; fi
  REAL=$(readlink -f "$TARGET")
  if [[ "$REAL" == *ttyUSB0 || "$REAL" == *ttyACM0 ]]; then
    echo "ERROR: 目标解析到 $REAL (疑似别的板子) 拒绝烧录"; exit 1; fi
  echo "=== 烧录到 $TARGET ($REAL) ==="
  python3 "$ESPTOOL" --chip esp32 --port "$TARGET" --baud 1500000 \
    --before default_reset --after hard_reset write_flash -z \
    --flash_mode dio --flash_freq 80m --flash_size 4MB \
    0x1000  "$BUILD_DIR/firmware.ino.bootloader.bin" \
    0x8000  "$BUILD_DIR/firmware.ino.partitions.bin" \
    0xe000  "$BOOT_APP" \
    0x10000 "$OUT_DIR/$BIN_NAME"
  echo "=== 烧录完成 ==="
fi
