#pragma once

#if defined(BUILD_FOR_ANDROID) || defined(__ANDROID__)
#include <android/log.h>

#define LOG_TAG "AdasApp"

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)

#else
#include <cstdio>

#define LOG_TAG "AdasApp"

// The do/while makes each macro a single statement. Without it the three calls below are three
// statements, so `if (cond) LOGW(...)` without braces printed the newline and flushed on every pass,
// not only when the condition held.
#define LOGI(...)                                                                                                      \
  do {                                                                                                                 \
    printf("[INFO] " LOG_TAG ": " __VA_ARGS__);                                                                        \
    printf("\n");                                                                                                      \
    fflush(stdout);                                                                                                    \
  } while (0)
#define LOGE(...)                                                                                                      \
  do {                                                                                                                 \
    printf("[ERROR] " LOG_TAG ": " __VA_ARGS__);                                                                       \
    printf("\n");                                                                                                      \
    fflush(stdout);                                                                                                    \
  } while (0)
#define LOGD(...)                                                                                                      \
  do {                                                                                                                 \
    printf("[DEBUG] " LOG_TAG ": " __VA_ARGS__);                                                                       \
    printf("\n");                                                                                                      \
    fflush(stdout);                                                                                                    \
  } while (0)
#define LOGW(...)                                                                                                      \
  do {                                                                                                                 \
    printf("[WARN] " LOG_TAG ": " __VA_ARGS__);                                                                        \
    printf("\n");                                                                                                      \
    fflush(stdout);                                                                                                    \
  } while (0)
#define LOGV(...)                                                                                                      \
  do {                                                                                                                 \
    printf("[VERBOSE] " LOG_TAG ": " __VA_ARGS__);                                                                     \
    printf("\n");                                                                                                      \
    fflush(stdout);                                                                                                    \
  } while (0)

#endif
