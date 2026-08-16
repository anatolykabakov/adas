ADAS assets (app/src/main/assets)

  config.json         — node feature flags + vehicle + camera calib priors
  supercombo.onnx     — vision model, supercombo 0.9.7 (7 inputs, output 6504, fp32)
  supercombo.thneed   — the same model compiled for the phone GPU, in fp16; built from
                        supercombo.onnx by scripts/tools/thneed_from_onnx.py --half,
                        byte-for-byte reproducible. See docs/THNEED.md
  vw_mqb_2010.dbc     — Golf 7 / MQB CAN DB
  toyota_nodsu_pt_generated.dbc — Toyota CAN DB

One model, two runners. thneed is the fast path (32 ms per frame on Adreno 640), ONNX the portable
fallback (55 ms). Both must stay the same network: the output layout differs between model
generations, so mixing them moves the pose and the lane lines to the wrong offsets without any
error. Measured agreement on the same frame: mean -1.4912 vs -1.4988, std 3.0224 vs 3.0345.

Why the ONNX asset is fp32 while the thneed is fp16: onnxruntime gets the fp16 model wrong on ARM.
On zero inputs the phone returns mean -7.6 / std 133.9 where the same file on a desktop returns
-1.25 / 3.28 — same on the CPU and the NNAPI provider, no error either way. SupercomboOnnxRunner
therefore runs every candidate session on zero inputs and refuses the ones whose signature does not
match the offline reference; that is how the NNAPI provider gets rejected here.

Build packaging (app/build.gradle → syncSupercomboModel):
  models/supercombo_097.onnx  →  assets/supercombo.onnx
  Override:  ./gradlew assembleDebug -PsupercomboModel=/path/to.onnx
  If models/ missing, keeps whatever is already in assets/.

Model load order (both runners):
  1) /sdcard/adas_models/<name>   (optional override, handy for measuring without a rebuild)
  2) app filesDir cache (ONNX only)
  3) assets/

Optional push (skip rebuild):
  adb shell mkdir -p /sdcard/adas_models
  adb push app/src/main/assets/supercombo.thneed /sdcard/adas_models/supercombo.thneed
