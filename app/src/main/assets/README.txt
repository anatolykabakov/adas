ADAS assets (app/src/main/assets)

  config.json       — node feature flags + vehicle + camera calib priors
  supercombo.onnx   — vision model (synced at build from models/sc_v0.8.13.onnx)
  vw_mqb_2010.dbc   — Golf 7 / MQB CAN DB

Build packaging (app/build.gradle → syncSupercomboModel):
  models/sc_v0.8.13.onnx  →  assets/supercombo.onnx
  Override:  ./gradlew assembleDebug -PsupercomboModel=/path/to.onnx
  If models/ missing, keeps whatever is already in assets/.

Model load order (SupercomboOnnxRunner):
  1) /sdcard/adas_models/supercombo.onnx   (optional override)
  2) app filesDir cache
  3) assets/supercombo.onnx

Optional push (skip rebuild):
  adb shell mkdir -p /sdcard/adas_models
  adb push models/sc_v0.8.13.onnx /sdcard/adas_models/supercombo.onnx
