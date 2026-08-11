#include <jni.h>
#include <cstdint>
#include <memory>
#include <string>

#include "adas/adas_app.h"
#include <json/json.h>

#include "adas/utils/logger.h"
#include "adas/utils/yuv_warp.h"

static std::unique_ptr<AdasApp> adas_app;

static std::string jstringToStd(JNIEnv* env, jstring value)
{
  if (!value)
    return {};
  const char* chars = env->GetStringUTFChars(value, nullptr);
  std::string out = chars ? chars : "";
  if (chars)
    env->ReleaseStringUTFChars(value, chars);
  return out;
}

extern "C" {

JNIEXPORT void JNICALL Java_adas_app_AdasAppHandler_nativeStart(JNIEnv* env, jclass, jint fd, jstring dbcPath,
                                                                jstring configPath, jstring mapPath)
{
  const std::string dbc_path = jstringToStd(env, dbcPath);
  const std::string config_path = jstringToStd(env, configPath);
  const std::string map_path = jstringToStd(env, mapPath);

  bool cfg_ok = false;
  AdasApp::Config cfg = AdasApp::Config::loadFromFile(config_path, &cfg_ok);
  if (!cfg_ok) {
    LOGW("JNI nativeStart: config load failed (%s), continuing with defaults", config_path.c_str());
  }

  // An APK asset has no path until Java unpacks it, so `map.path` in the config names the asset and this is
  // where it landed. Empty means the `map_data` node is off and Java did not unpack it — leave the config
  // value alone so a hand-pushed `/sdcard/adas_maps/` copy still works.
  if (!map_path.empty()) {
    cfg.map_data.map_path = map_path;
  }

  LOGI("JNI nativeStart fd=%d dbc=%s config=%s map=%s lane_keep=%d loc=%d map_data=%d", fd, dbc_path.c_str(),
       config_path.c_str(), cfg.map_data.map_path.c_str(), cfg.feature_flags.enable_lane_keep ? 1 : 0,
       cfg.feature_flags.enable_localization ? 1 : 0, cfg.feature_flags.enable_map_data ? 1 : 0);
  try {
    if (!adas_app) {
      adas_app = std::make_unique<AdasApp>(fd, dbc_path, cfg);
    }
    if (!adas_app->start()) {
      LOGE("AdasApp failed to start");
    }
  } catch (const std::exception& e) {
    LOGE("Exception in JNI nativeStart: %s", e.what());
  } catch (...) {
    LOGE("Unknown exception in JNI nativeStart");
  }
}

JNIEXPORT void JNICALL Java_adas_app_AdasAppHandler_nativeStop(JNIEnv*, jclass)
{
  if (adas_app) {
    adas_app->stop();
    adas_app.reset();
  }
}

JNIEXPORT jint JNICALL Java_adas_app_AdasAppHandler_nativeUpdateParams(JNIEnv* env, jclass, jstring jsonParams)
{
  if (!adas_app)
    return -1;
  if (jsonParams == nullptr)
    return 0;
  const std::string text = jstringToStd(env, jsonParams);
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errs;
  std::istringstream in(text);
  if (!Json::parseFromStream(builder, in, &root, &errs) || !root.isObject()) {
    LOGE("nativeUpdateParams: not a JSON object (%s)", errs.c_str());
    return 0;
  }

  std::map<std::string, std::string> params;
  for (const auto& name : root.getMemberNames()) {
    const Json::Value& v = root[name];
    if (v.isString())
      params[name] = v.asString();
    else if (v.isBool())
      params[name] = v.asBool() ? "true" : "false";
    else if (v.isNumeric())
      params[name] = std::to_string(v.asDouble());
    else
      LOGW("nativeUpdateParams: skipping '%s' — not a scalar", name.c_str());
  }
  return static_cast<jint>(adas_app->updateParams(params));
}

JNIEXPORT jboolean JNICALL Java_adas_app_vision_ModelCalibWarp_nativeWarpYuvToFrame6(JNIEnv* env, jclass,
                                                                                     jbyteArray yArr, jbyteArray uArr,
                                                                                     jbyteArray vArr, jint width,
                                                                                     jint height, jfloatArray mArr,
                                                                                     jfloatArray outArr)
{
  if (!yArr || !uArr || !vArr || !mArr || !outArr || width <= 0 || height <= 0)
    return JNI_FALSE;

  const jint y_len = env->GetArrayLength(yArr);
  const jint u_len = env->GetArrayLength(uArr);
  const jint v_len = env->GetArrayLength(vArr);
  const jint m_len = env->GetArrayLength(mArr);
  const jint out_len = env->GetArrayLength(outArr);
  const int uv_w = width / 2;
  const int uv_h = height / 2;
  const int plane = (adas::kModelW / 2) * (adas::kModelH / 2);
  if (y_len < width * height || u_len < uv_w * uv_h || v_len < uv_w * uv_h || m_len < 9 || out_len < 6 * plane)
    return JNI_FALSE;

  jbyte* y = env->GetByteArrayElements(yArr, nullptr);
  jbyte* u = env->GetByteArrayElements(uArr, nullptr);
  jbyte* v = env->GetByteArrayElements(vArr, nullptr);
  jfloat* m = env->GetFloatArrayElements(mArr, nullptr);
  jfloat* out = env->GetFloatArrayElements(outArr, nullptr);
  if (!y || !u || !v || !m || !out) {
    if (y)
      env->ReleaseByteArrayElements(yArr, y, JNI_ABORT);
    if (u)
      env->ReleaseByteArrayElements(uArr, u, JNI_ABORT);
    if (v)
      env->ReleaseByteArrayElements(vArr, v, JNI_ABORT);
    if (m)
      env->ReleaseFloatArrayElements(mArr, m, JNI_ABORT);
    if (out)
      env->ReleaseFloatArrayElements(outArr, out, JNI_ABORT);
    return JNI_FALSE;
  }

  adas::warp_yuv_to_frame6(reinterpret_cast<const uint8_t*>(y), reinterpret_cast<const uint8_t*>(u),
                           reinterpret_cast<const uint8_t*>(v), width, height, m, out);

  env->ReleaseByteArrayElements(yArr, y, JNI_ABORT);
  env->ReleaseByteArrayElements(uArr, u, JNI_ABORT);
  env->ReleaseByteArrayElements(vArr, v, JNI_ABORT);
  env->ReleaseFloatArrayElements(mArr, m, JNI_ABORT);
  env->ReleaseFloatArrayElements(outArr, out, 0);
  return JNI_TRUE;
}

}  // extern "C"
