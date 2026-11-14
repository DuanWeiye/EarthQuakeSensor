#pragma once
// 拷贝本文件为 config.h 并填真实值
#define WIFI_SSID   "YOUR_SSID"
#define WIFI_PASS   "YOUR_PASS"
// REPORT_BASE = 上报 API 的完整基址(末尾不带 /)。设备在局域网里, 用明文 http 并复用一条 keep-alive 连接
// (每次新建 TLS 握手的空口占用会拖慢同 AP 的其它终端); 服务器 keep-alive 超时要大于心跳间隔 15s。
//   POST 此地址             = 上报事件
//   POST 此地址 + "/wave?…"    = 原始波形块
//   GET  此地址 + "/config"    = 拉取检测参数(可附带 OTA 版本广播)
//   POST 此地址 + "/heartbeat" = 心跳(带遥测, 响应带回检测参数)
#define REPORT_BASE "http://192.168.1.10/api"
// OTA_BASE = OTA 下载的基址(末尾不带 /), 走 https:
//   GET  此地址 + "/firmware"  = 新固件 .bin
#define OTA_BASE    "https://example.com/api"
