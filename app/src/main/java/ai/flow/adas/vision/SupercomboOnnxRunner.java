package ai.flow.adas.vision;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

import ai.flow.adas.AdasConfig;
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtLoggingLevel;
import ai.onnxruntime.OrtSession;

public class SupercomboOnnxRunner {
    private static final String TAG = "SupercomboOnnx";

    public static final int MODEL_W = 512;
    public static final int MODEL_H = 256;
    public static final int CH_PER_FRAME = 6;
    public static final int TENSOR_C = 12;
    public static final int TENSOR_H = 128;
    public static final int TENSOR_W = 256;


    private static final int PLAN_END = 4955;
    // Lanes slice: first 264 floats are the means (4 × 33 × y,z), the next 264 the log-sigmas
    // in the same layout. See app/src/main/scripts/core/supercombo_parse.py for the full map.
    private static final int LANE_STDS_START = PLAN_END + 264;
    private static final int LANES_END = PLAN_END + 528;
    private static final int LANE_PROB_END = LANES_END + 8;
    private static final int ROAD_END = LANE_PROB_END + 264;

    private static final int PLAN_MHP_N = 5;
    private static final int PLAN_COLS = 15;

    private static final int PLAN_GROUP = 2 * PLAN_COLS * LaneLines.N + 1;

    private final OrtEnvironment env;
    private final OrtSession session;
    private final float[] desire = new float[8];
    private final float[] traffic = new float[2];
    private final float[] rnnState = new float[512];

    private float[] prevFrame6;
    private boolean hasPrev;

    /** Model←camera warp (model→camera homography); rebuilt via {@link #setCalib}. */
    private float[] warpM = ModelCalibWarp.warpMatrixDeg(0, 0, 0, 930f, 930f, 640f, 360f);
    private float rollDeg;
    private float pitchDeg;
    private float yawDeg;
    private float fx = 930f;
    private float fy = 930f;
    private float cx = 640f;
    private float cy = 360f;
    private int calibW = 1280;
    private int calibH = 720;

    private final int[] resizePixels = new int[MODEL_W * MODEL_H];
    private final byte[] yuvI420 = new byte[MODEL_W * MODEL_H * 3 / 2];
    private final float[] currFrame6 = new float[CH_PER_FRAME * TENSOR_H * TENSOR_W];
    private final float[] input12 = new float[TENSOR_C * TENSOR_H * TENSOR_W];

    public static final class Result {
        public final LaneLines lanes;
        public final CameraOdometry pose;
        public final ModelLongParse.Out modelLong;

        public Result(LaneLines lanes, CameraOdometry pose, ModelLongParse.Out modelLong) {
            this.lanes = lanes;
            this.pose = pose;
            this.modelLong = modelLong;
        }
    }

    public SupercomboOnnxRunner(Context context) throws Exception {
        env = OrtEnvironment.getEnvironment();
        File model = resolveModelFile(context);
        Log.i(TAG, "Loading model: " + model.getAbsolutePath() + " (" + model.length() + " bytes)"
                + " availableProviders=" + env.getAvailableProviders());

        session = createSessionPreferNnapi(model.getAbsolutePath());

        traffic[0] = 1.f;
        traffic[1] = 0.f;
        Log.i(TAG, "ONNX ready inputs=" + session.getInputNames()
                + " outputs=" + session.getOutputNames());
    }

    /**
     * Prefer Android NNAPI (GPU/DSP/NPU). On OnePlus 7T ~35 ms vs ~107 ms CPU.
     * Falls back to CPU if NNAPI session create fails.
     */
    private OrtSession createSessionPreferNnapi(String modelPath) throws Exception {
        OrtSession.SessionOptions opts = newSessionOptions(/*threads=*/ 2);
        try {
            opts.addNnapi();
            OrtSession s = env.createSession(modelPath, opts);
            Log.i(TAG, "NNAPI EP enabled");
            return s;
        } catch (Throwable t) {
            Log.w(TAG, "NNAPI session failed — falling back to CPU", t);
            try {
                opts.close();
            } catch (Throwable ignored) {
            }
        }
        OrtSession.SessionOptions cpuOpts = newSessionOptions(/*threads=*/ 2);
        return env.createSession(modelPath, cpuOpts);
    }

    private static OrtSession.SessionOptions newSessionOptions(int intraOpThreads) throws Exception {
        OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
        opts.setIntraOpNumThreads(Math.max(1, intraOpThreads));
        try {
            opts.setSessionLogLevel(OrtLoggingLevel.ORT_LOGGING_LEVEL_WARNING);
        } catch (Throwable ignored) {
        }
        return opts;
    }

    /**
     * Update calib warp from UI / config (degrees + camera K for capture resolution).
     * Resets temporal pair so the next frame re-seeds the stack.
     */
    public synchronized void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                                      float fx, float fy, float cx, float cy,
                                      int width, int height) {
        this.rollDeg = rollDeg;
        this.pitchDeg = pitchDeg;
        this.yawDeg = yawDeg;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.calibW = Math.max(1, width);
        this.calibH = Math.max(1, height);
        this.warpM = ModelCalibWarp.warpMatrixDeg(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy);
        this.hasPrev = false;
        this.prevFrame6 = null;
        Log.i(TAG, String.format(
                "calib warp rpy_deg=(%.2f,%.2f,%.2f) K=(%.1f,%.1f,%.1f,%.1f) %dx%d",
                rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, calibW, calibH));
    }

    public synchronized void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                                      float fx, float fy, float cx, float cy) {
        setCalib(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, calibW, calibH);
    }

    private static File resolveModelFile(Context context) throws Exception {
        String assetName = "supercombo.onnx";
        try {
            assetName = AdasConfig.supercomboAsset(context);
        } catch (Throwable ignored) {
        }
        File external = new File("/sdcard/adas_models/" + assetName);
        if (external.exists() && external.length() > 1_000_000) {
            return external;
        }
        File cached = new File(context.getFilesDir(), assetName);

        long assetLen = -1;
        try (InputStream in = context.getAssets().open(assetName)) {
            assetLen = in.available();
        } catch (Exception ignored) {
        }
        boolean needCopy = !cached.exists() || cached.length() < 1_000_000
                || (assetLen > 1_000_000 && Math.abs(cached.length() - assetLen) > 1024);
        if (!needCopy) {
            return cached;
        }
        try (InputStream in = context.getAssets().open(assetName);
             FileOutputStream out = new FileOutputStream(cached)) {
            byte[] buf = new byte[1 << 20];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
            }
            return cached;
        } catch (Exception e) {
            throw new IllegalStateException(
                    assetName + " not found. Push with:\n" +
                    "  adb shell mkdir -p /sdcard/adas_models\n" +
                    "  adb push /path/to/supercombo.onnx /sdcard/adas_models/supercombo.onnx",
                    e);
        }
    }


    public synchronized Result run(Bitmap frame, int frameId) throws Exception {
        return run(frame, frameId, ai.flow.adas.TimeUtil.nowMs());
    }

    /** Legacy ARGB path (CPU RGB warp). Prefer {@link #run(YuvFrame, int, long)}. */
    public synchronized Result run(Bitmap frame, int frameId, long captureTsMs) throws Exception {
        float[] m = resolveWarp(frame.getWidth(), frame.getHeight());
        Bitmap warped = ModelCalibWarp.warpToModel(frame, m);
        try {
            warped.getPixels(resizePixels, 0, MODEL_W, 0, 0, MODEL_W, MODEL_H);
            rgbToYuvI420(resizePixels, MODEL_W, MODEL_H, yuvI420);
            return runPreparedYuv(frameId, captureTsMs);
        } finally {
            warped.recycle();
        }
    }

    /**
     * Flowpilot-style path: warp Y/U/V with calib homography (no ARGB), then ONNX.
     */
    public synchronized Result run(YuvFrame frame, int frameId, long captureTsMs) throws Exception {
        float[] m = resolveWarp(frame.width, frame.height);
        long tPrep0 = System.nanoTime();
        ModelCalibWarp.warpYuvToFrame6(frame, m, currFrame6);
        float prepMs = (float) ((System.nanoTime() - tPrep0) / 1e6);
        return runPreparedFrame6(frameId, captureTsMs, prepMs);
    }

    private float[] resolveWarp(int frameW, int frameH) {
        float[] m = warpM;
        if (frameW != calibW || frameH != calibH) {
            float sx = frameW / (float) calibW;
            float sy = frameH / (float) calibH;
            m = ModelCalibWarp.warpMatrixDeg(
                    rollDeg, pitchDeg, yawDeg, fx * sx, fy * sy, cx * sx, cy * sy);
        }
        return m;
    }

    private Result runPreparedYuv(int frameId, long captureTsMs) throws Exception {
        long tPrep0 = System.nanoTime();
        parseImageYuvI420(yuvI420, MODEL_W, MODEL_H, currFrame6);
        float prepMs = (float) ((System.nanoTime() - tPrep0) / 1e6);
        return runPreparedFrame6(frameId, captureTsMs, prepMs);
    }

    private Result runPreparedFrame6(int frameId, long captureTsMs, float prepMs) throws Exception {
        if (!hasPrev) {
            prevFrame6 = currFrame6.clone();
            hasPrev = true;
            return null;
        }

        System.arraycopy(prevFrame6, 0, input12, 0, prevFrame6.length);
        System.arraycopy(currFrame6, 0, input12, prevFrame6.length, currFrame6.length);
        System.arraycopy(currFrame6, 0, prevFrame6, 0, currFrame6.length);

        long[] shape = new long[]{1, TENSOR_C, TENSOR_H, TENSOR_W};
        try (OnnxTensor tImgs = OnnxTensor.createTensor(env, FloatBuffer.wrap(input12), shape);
             OnnxTensor tDesire = OnnxTensor.createTensor(env, FloatBuffer.wrap(desire), new long[]{1, 8});
             OnnxTensor tTraffic = OnnxTensor.createTensor(env, FloatBuffer.wrap(traffic), new long[]{1, 2});
             OnnxTensor tState = OnnxTensor.createTensor(env, FloatBuffer.wrap(rnnState), new long[]{1, 512})) {

            Map<String, OnnxTensor> feeds = new HashMap<>();
            feeds.put("input_imgs", tImgs);
            feeds.put("desire", tDesire);
            feeds.put("traffic_convention", tTraffic);
            feeds.put("initial_state", tState);

            long tInfer0 = System.nanoTime();
            try (OrtSession.Result result = session.run(feeds)) {
                float inferMs = (float) ((System.nanoTime() - tInfer0) / 1e6);
                Object value = result.get(0).getValue();
                float[] flat = flattenOutput(value);
                LaneLines lanes = parseLanes(flat);
                lanes.frameId = frameId;
                long capture = captureTsMs > 0 ? captureTsMs : ai.flow.adas.TimeUtil.nowMs();
                long infer = ai.flow.adas.TimeUtil.nowMs();
                lanes.captureTimestampMs = capture;
                lanes.inferTimestampMs = infer;
                lanes.inferDurationMs = inferMs;
                lanes.prepDurationMs = prepMs;
                lanes.timestampMs = capture;
                lanes.modelOut = flat;
                CameraOdometry pose = CameraOdometry.parse(flat);
                ModelLongParse.Out modelLong = ModelLongParse.parse(flat);
                if (modelLong != null && modelLong.ok) {
                    ModelLongParse.Lead best =
                            modelLong.lead0.prob >= modelLong.lead1.prob ? modelLong.lead0 : modelLong.lead1;
                    if (modelLong.lead2 != null && modelLong.lead2.prob > best.prob) {
                        best = modelLong.lead2;
                    }
                    lanes.leadD = best.x[0];
                    lanes.leadY = best.y[0];
                    lanes.leadV = best.v[0];
                    lanes.leadProb = best.prob;
                    lanes.planV0 = modelLong.planVx[0];
                    lanes.leadValid = best.prob >= 0.4f && best.x[0] > 1.f && best.x[0] < 120.f;
                }

                int poseIdx = CameraOdometry.poseIdx(flat.length);
                if (flat.length >= poseIdx + CameraOdometry.POSE_SIZE
                        + CameraOdometry.TEMPORAL_SIZE) {
                    System.arraycopy(flat, flat.length - CameraOdometry.TEMPORAL_SIZE,
                            rnnState, 0, CameraOdometry.TEMPORAL_SIZE);
                } else if (flat.length >= ROAD_END + 512) {
                    System.arraycopy(flat, flat.length - 512, rnnState, 0, 512);
                }
                return new Result(lanes, pose, modelLong);
            }
        }
    }

    private static float[] flattenOutput(Object value) {
        if (value instanceof float[][]) {
            float[][] a = (float[][]) value;
            return a[0];
        }
        if (value instanceof float[]) {
            return (float[]) value;
        }
        throw new IllegalStateException("Unexpected ONNX output type: " + value.getClass());
    }

    private static LaneLines parseLanes(float[] out) {
        LaneLines ll = new LaneLines();
        if (out.length < ROAD_END) {
            Log.w(TAG, "Output too short: " + out.length);
            return ll;
        }


        int bestHyp = 0;
        float bestLogit = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < PLAN_MHP_N; i++) {
            float logit = out[(i + 1) * PLAN_GROUP - 1];
            if (logit > bestLogit) {
                bestLogit = logit;
                bestHyp = i;
            }
        }
        int planBase = bestHyp * PLAN_GROUP;
        for (int i = 0; i < LaneLines.N; i++) {
            int row = planBase + i * PLAN_COLS;
            ll.planX[i] = out[row];

            // Device frame (flowpilot/openpilot): X forward, Y right-positive, Z up.
            ll.planY[i] = out[row + 1];
            ll.planZ[i] = out[row + 2];
            // orientation.z / orientationRate.z (Parser.fillXYZT column_offset 9 / 12 → z = +2)
            ll.planYaw[i] = out[row + 11];
            ll.planYawRate[i] = out[row + 14];
        }
        ll.planHypIndex = bestHyp;
        ll.hasPlan = true;


        for (int lane = 0; lane < 4; lane++) {
            int base = PLAN_END + lane * 66;
            int stdBase = LANE_STDS_START + lane * 66;
            for (int i = 0; i < LaneLines.N; i++) {
                ll.lanesY[lane][i] = out[base + i * 2];
                ll.lanesZ[lane][i] = out[base + i * 2 + 1];
                ll.lanesYStd[lane][i] = (float) Math.exp(out[stdBase + i * 2]);
            }

            ll.laneProbs[lane] = sigmoid(out[LANES_END + lane * 2 + 1]);
        }


        for (int edge = 0; edge < 2; edge++) {
            int base = LANE_PROB_END + edge * 66;
            for (int i = 0; i < LaneLines.N; i++) {
                ll.edgesY[edge][i] = out[base + i * 2];
                ll.edgesZ[edge][i] = out[base + i * 2 + 1];
            }
        }
        return ll;
    }

    private static float sigmoid(float x) {
        return (float) (1.0 / (1.0 + Math.exp(-x)));
    }


    static void rgbToYuvI420(int[] argb, int w, int h, byte[] out) {
        int ySize = w * h;
        int uvW = w / 2;
        int uvH = h / 2;
        int uOff = ySize;
        int vOff = ySize + uvW * uvH;
        int yi = 0;
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                int c = argb[yi];
                int r = (c >> 16) & 0xff;
                int g = (c >> 8) & 0xff;
                int b = c & 0xff;
                int y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
                out[yi] = (byte) clamp(y, 0, 255);
                if ((j % 2 == 0) && (i % 2 == 0)) {
                    int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
                    int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
                    int uvi = (j / 2) * uvW + (i / 2);
                    out[uOff + uvi] = (byte) clamp(u, 0, 255);
                    out[vOff + uvi] = (byte) clamp(v, 0, 255);
                }
                yi++;
            }
        }
    }


    static void parseImageYuvI420(byte[] frame, int w, int h, float[] out6) {
        int H = (frame.length * 2) / 3 / w;
        if (H != h) {
            H = h;
        }
        int hh = H / 2;
        int ww = w / 2;
        int plane = hh * ww;

        for (int j = 0; j < hh; j++) {
            for (int i = 0; i < ww; i++) {
                int y00 = frame[(2 * j) * w + (2 * i)] & 0xff;
                int y10 = frame[(2 * j + 1) * w + (2 * i)] & 0xff;
                int y01 = frame[(2 * j) * w + (2 * i + 1)] & 0xff;
                int y11 = frame[(2 * j + 1) * w + (2 * i + 1)] & 0xff;
                int idx = j * ww + i;
                out6[0 * plane + idx] = y00;
                out6[1 * plane + idx] = y10;
                out6[2 * plane + idx] = y01;
                out6[3 * plane + idx] = y11;
            }
        }
        int uOff = H * w;
        int vOff = uOff + hh * ww;
        for (int i = 0; i < plane; i++) {
            out6[4 * plane + i] = frame[uOff + i] & 0xff;
            out6[5 * plane + i] = frame[vOff + i] & 0xff;
        }
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void close() {
        try {
            session.close();
        } catch (Exception ignored) {
        }
    }
}
