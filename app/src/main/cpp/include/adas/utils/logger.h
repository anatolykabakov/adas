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

#define LOGI(...)                                                                                                      \
  printf("[INFO] " LOG_TAG ": " __VA_ARGS__);                                                                          \
  printf("\n");                                                                                                        \
  fflush(stdout)
#define LOGE(...)                                                                                                      \
  printf("[ERROR] " LOG_TAG ": " __VA_ARGS__);                                                                         \
  printf("\n");                                                                                                        \
  fflush(stdout)
#define LOGD(...)                                                                                                      \
  printf("[DEBUG] " LOG_TAG ": " __VA_ARGS__);                                                                         \
  printf("\n");                                                                                                        \
  fflush(stdout)
#define LOGW(...)                                                                                                      \
  printf("[WARN] " LOG_TAG ": " __VA_ARGS__);                                                                          \
  printf("\n");                                                                                                        \
  fflush(stdout)
#define LOGV(...)                                                                                                      \
  printf("[VERBOSE] " LOG_TAG ": " __VA_ARGS__);                                                                       \
  printf("\n");                                                                                                        \
  fflush(stdout)

#endif
