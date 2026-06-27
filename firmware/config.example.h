#pragma once
// 拷贝本文件为 config.h 并填真实值
#define WIFI_SSID   "YOUR_SSID"
#define WIFI_PASS   "YOUR_PASS"
// REPORT_BASE = 事件上报 API 的完整基址(末尾不带 /):
//   POST 此地址             = 上报事件
//   GET  此地址 + "/config"    = 拉取检测参数
//   POST 此地址 + "/heartbeat" = 心跳(带遥测)
#define REPORT_BASE "https://example.com/api"
