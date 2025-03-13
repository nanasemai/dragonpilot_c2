#pragma once

#include "common/timing.h"

#define CLOUDLOG_DEBUG 10
#define CLOUDLOG_INFO 20
#define CLOUDLOG_WARNING 30
#define CLOUDLOG_ERROR 40
#define CLOUDLOG_CRITICAL 50


void cloudlog_e(int levelnum, const char* filename, int lineno, const char* func,
                const char* fmt, ...) /*__attribute__ ((format (printf, 6, 7)))*/;

void cloudlog_te(int levelnum, const char* filename, int lineno, const char* func,
                 const char* fmt, ...) /*__attribute__ ((format (printf, 6, 7)))*/;

void cloudlog_te(int levelnum, const char* filename, int lineno, const char* func,
                 uint32_t frame_id, const char* fmt, ...) /*__attribute__ ((format (printf, 6, 7)))*/;


#define cloudlog(lvl, fmt, ...) cloudlog_e(lvl, __FILE__, __LINE__, \
                                           __func__, \
                                           fmt, ## __VA_ARGS__)

#define cloudlog_t(lvl, ...) cloudlog_te(lvl, __FILE__, __LINE__, \
                                          __func__, \
                                          __VA_ARGS__)


#define cloudlog_rl(burst, millis, lvl, fmt, ...)   \
{                                                   \
  static uint64_t __begin = 0;                      \
  static int __printed = 0;                         \
  static int __missed = 0;                          \
                                                    \
  int __burst = (burst);                            \
  int __millis = (millis);                          \
  uint64_t __ts = nanos_since_boot();               \
                                                    \
  if (!__begin) { __begin = __ts; }                 \
                                                    \
  if (__begin + __millis*1000000ULL < __ts) {       \
    if (__missed) {                                 \
      cloudlog(CLOUDLOG_WARNING, "cloudlog: %d messages suppressed", __missed); \
    }                                               \
    __begin = 0;                                    \
    __printed = 0;                                  \
    __missed = 0;                                   \
  }                                                 \
                                                    \
  if (__printed < __burst) {                        \
    cloudlog(lvl, fmt, ## __VA_ARGS__);             \
    __printed++;                                    \
  } else {                                          \
    __missed++;                                     \
  }                                                 \
}


#define LOGT(...) cloudlog_t(CLOUDLOG_DEBUG, __VA_ARGS__)
#define LOGD(fmt, ...) cloudlog(CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG(fmt, ...) cloudlog(CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW(fmt, ...) cloudlog(CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE(fmt, ...) cloudlog(CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)

#define LOGD_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE_100(fmt, ...) cloudlog_rl(2, 100, CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)


// 添加模块上下文支持
void cloudlog_bind(const char* key, const char* value);

// 添加新的带上下文的日志函数
void cloudlog_ec(int levelnum, const char* filename, int lineno, const char* func,
                const char* fmt, ...) /*__attribute__ ((format (printf, 6, 7)))*/;

// 新增带上下文的宏定义
#define cloudlog_c(lvl, fmt, ...) cloudlog_ec(lvl, __FILE__, __LINE__, \
                                           __func__, \
                                           fmt, ## __VA_ARGS__)

// 新增便捷宏
#define LOGD_C(fmt, ...) cloudlog_c(CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG_C(fmt, ...) cloudlog_c(CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW_C(fmt, ...) cloudlog_c(CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE_C(fmt, ...) cloudlog_c(CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)
