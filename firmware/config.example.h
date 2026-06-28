#pragma once
// 拷贝本文件为 config.h 并填真实值
#define WIFI_SSID   "YOUR_SSID"
#define WIFI_PASS   "YOUR_PASS"
// REPORT_BASE = 事件上报 API 的完整基址(末尾不带 /):
//   POST 此地址             = 上报事件
//   GET  此地址 + "/config"    = 拉取检测参数(可附带 OTA 版本广播)
//   POST 此地址 + "/heartbeat" = 心跳(带遥测)
//   GET  此地址 + "/firmware"  = OTA 下载新固件 .bin
#define REPORT_BASE "https://example.com/api"
