package adas.app.vision;

import adas.app.TimeUtil;
import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

import adas.app.AdasConfig;
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtLoggingLevel;
import ai.onnxruntime.OrtSession;
import ai.onnxruntime.providers.NNAPIFlags;

public class SupercomboOnnxRunner implements ModelRunner {

    @Override
    public String name() {
        return "onnx";
    }

    private static final String TAG = "SupercomboOnnx";

    public static final int MODEL_W = 512;
    public static final int MODEL_H = 256;
    public static final int CH_PER_FRAME = 6;
    public static final int TENSOR_C = 12;
    public static final int TENSOR_H = 128;
    public static final int TENSOR_W = 256;


    private static final int PLAN_END = 4955;
    // Lanes slice: first 264 floats are the means (4 × 33 × y,z), the next 264 the log-sigmas
    // in the same layout. See scripts/core/supercombo_parse.py for the full map.
    private static final int LANE_STDS_START = PLAN_END + 264;
    private static final int LANES_END = PLAN_END + 528;
    private static final int LANE_PROB_END = LANES_END + 8;
    // Road edges: same two-half layout as the lanes — 132 floats of means (2 × 33 × y,z), then 132
    // of log-sigmas. The sigmas were parsed by nobody until 2026-08-06, which is why the road-edge
    // fallback for single-line stretches could not be evaluated: flowpilot gates edges on their
    // sigma, and ours were not even recorded.
    private static final int EDGE_STDS_START = LANE_PROB_END + 132;
    private static final int ROAD_END = LANE_PROB_END + 264;

    private static final int PLAN_MHP_N = 5;
    private static final int PLAN_COLS = 15;

    private static final int PLAN_GROUP = 2 * PLAN_COLS * LaneLines.N + 1;

    // supercombo 0.9.7 dimensions. The same numbers are baked into the native thneed runner
    // (`thneed_runner.cpp`): both paths now compute **one and the same network** and must not drift.
    /** How many frames of features the model remembers. */
    private static final int HISTORY_LEN = 99;
    /** Features per frame — the model emits them in the output tail and takes them back as input. */
    private static final int FEATURE_LEN = 512;
    /** Length of the parsed part of the output; the feedback features follow it. */
    private static final int PARSED_OUTPUT = 5992;
    /** Full output length: the parsed part plus the features. */
    public static final int NET_OUTPUT_SIZE = PARSED_OUTPUT + FEATURE_LEN;
    /** Desired-curvature history: as many frames as features, plus the current one. */
    private static final int PREV_CURV_LEN = HISTORY_LEN + 1;
    /** Manoeuvre desire: 100 frames of 8 features. We leave it empty — there is no lane-change planner. */
    private static final int DESIRE_LEN = 800;
    /** Where the desired curvature sits in the output — that is what feeds back into the input. */
    private static final int DESIRED_CURV_IDX = 5990;
    /** Camera-pose offset in the 0.9.x output; the same value lives in the thneed runner. */
    private static final int POSE_IDX_09X = 5948;

    private final OrtEnvironment env;
    private final OrtSession session;
    /** Input precision of the accepted session — a property of the model file, not of the code. */
    private final boolean halfPrecision;
    private final float[] desire = new float[DESIRE_LEN];
    private final float[] traffic = new float[2];
    private final float[] latParams = new float[2];

    /**
     * Features of past frames that the model feeds back to itself.
     *
     * <p>The recurrence is not hidden inside the network but exposed: every frame the history shifts
     * by one and the output's features are appended to the tail. Skipping that shift means feeding the
     * model the same past over and over, and it will drive on lane lines that are no longer there.
     */
    private final float[] featuresBuffer = new float[HISTORY_LEN * FEATURE_LEN];
    /** Desired-curvature history — the second feedback path, shifted one value at a time. */
    private final float[] prevDesiredCurv = new float[PREV_CURV_LEN];

    private float[] prevFrame6;
    private float[] prevWide6;
    private boolean hasPrev;
    private volatile float egoSpeedMps;

    /** Model←camera warp (model→camera homography); rebuilt via {@link #setCalib}. */
    private float[] warpM = ModelCalibWarp.warpMatrixDeg(0, 0, 0, 930f, 930f, 640f, 360f, false);
    /** The second matrix, for the model's wide input. */
    private float[] warpWideM = ModelCalibWarp.warpMatrixDeg(0, 0, 0, 930f, 930f, 640f, 360f, true);
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
    // The wide frame. We have one camera, so the same frame goes through a second matrix — exactly
    // what the thneed path does.
    private final float[] currWide6 = new float[CH_PER_FRAME * TENSOR_H * TENSOR_W];
    private final float[] inputWide12 = new float[TENSOR_C * TENSOR_H * TENSOR_W];
    private final float[] netOutput = new float[NET_OUTPUT_SIZE];

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

        final float[] signature = AdasConfig.zeroInputSignature(context, "onnx");
        if (signature != null) {
            zeroInputMean = signature[0];
            zeroInputStd = signature[1];
        }
        session = createValidatedSession(model.getAbsolutePath(), AdasConfig.nnapiFp16(context));

        // traffic_convention stays zero — exactly as in the native thneed path. A meaningful value
        // ("right-hand traffic" = {1, 0}) is worth setting, but for both paths at once and with a
        // separate check on the road: right now it matters more that they compute alike.
        halfPrecision = isHalfPrecision(session);
        Log.i(TAG, "ONNX ready fp16=" + halfPrecision + " inputs=" + session.getInputNames()
                + " outputs=" + session.getOutputNames());
    }

    /**
     * Prefer Android NNAPI (GPU/DSP/NPU). On OnePlus 7T ~35 ms vs ~107 ms CPU.
     *
     * <p>Tries half precision first when asked ({@code vision.nnapi_fp16}, see
     * {@link AdasConfig#nnapiFp16} for the offline evidence and why it is off by default), then plain
     * NNAPI, then CPU. Each attempt builds its own {@code SessionOptions}: options cannot be reused
     * after a failed {@code createSession}, which is why {@code TrafficYoloRunner} does the same.
     */
    /**
     * Output signature on zero inputs, taken offline: same file, same zeros, `onnxruntime` from
     * Python on the CPU. Serves as acceptance: a session that answers differently computes the wrong
     * thing.
     */
    private static final float ZERO_INPUT_MEAN = -1.2455f;
    private static final float ZERO_INPUT_STD = 3.2799f;

    /** The same signature taken from the config when set there: it belongs to the model file. */
    private float zeroInputMean = ZERO_INPUT_MEAN;
    private float zeroInputStd = ZERO_INPUT_STD;
    /** A loose tolerance: we catch garbage, not half-precision rounding. */
    private static final float ZERO_INPUT_TOLERANCE = 0.5f;

    /**
     * Build a session and make sure it computes **the right thing**.
     *
     * <p>"The session was created" is not enough. NNAPI on this SoC takes the fp16 graph and returns
     * mean −7.6 with spread 134 on zero inputs instead of −1.25 and 3.3 — it does not crash, does not
     * complain, and silently computes something else. A road frame cannot tell: numbers look like
     * numbers.
     *
     * <p>So every built session first goes through a zero-input run, and if the signature disagrees
     * the accelerator is dropped and the next one is tried. Slow but right beats fast and wrong.
     */
    private OrtSession createValidatedSession(String modelPath, boolean fp16) throws Exception {
        // Ordered from fastest to most dependable. Every candidate goes through the zero-input
        // acceptance, so the list can grow without risk: whatever computes wrongly drops out itself.
        if (fp16) {
            OrtSession s = acceptIfSane(tryNnapi(modelPath, true), "NNAPI fp16");
            if (s != null) {
                return s;
            }
        }
        // NNAPI is tried in several configurations. On this phone every one of them computes the
        // wrong thing and acceptance drops it, but the configuration is a property of the driver, not
        // ours: on another device any of them may pass. Each variant costs one session, half a second.
        for (java.util.EnumSet<NNAPIFlags> flags : java.util.List.of(
                java.util.EnumSet.noneOf(NNAPIFlags.class),
                java.util.EnumSet.of(NNAPIFlags.USE_NCHW),
                java.util.EnumSet.of(NNAPIFlags.CPU_DISABLED))) {
            OrtSession nnapi = acceptIfSane(tryNnapiFlags(modelPath, flags), "NNAPI " + flags);
            if (nnapi != null) {
                return nnapi;
            }
        }
        OrtSession xnn = acceptIfSane(tryXnnpack(modelPath), "XNNPACK");
        if (xnn != null) {
            return xnn;
        }
        OrtSession cpu = acceptIfSane(tryCpu(modelPath), "CPU");
        if (cpu != null) {
            return cpu;
        }
        throw new IllegalStateException(
                "the model computes wrongly on every accelerator — see the zero-input signatures in the log");
    }

    /**
     * XNNPACK: the same CPU kernels, rewritten for ARM.
     *
     * <p>The provider is present on this phone and had never been tried — and it is exactly about what
     * is missing here: single-precision convolutions on NEON. It needs no accelerator, so it should
     * not break the output signature either, but it is checked like everything else.
     */
    /**
     * Return the session if its zero-input run matched the reference, otherwise close it and return
     * null.
     *
     * <p>"The session was created" and "the session computes the same thing" are different, and the
     * second is invisible in the log: on both phones checked, NNAPI runs without a single complaint
     * and returns foreign numbers.
     */
    private OrtSession acceptIfSane(OrtSession candidate, String what) {
        if (candidate == null) {
            return null;
        }
        try {
            requireGeneration09x(candidate.getInputNames());
            float[] signature = zeroInputSignature(candidate);
            boolean ok = Math.abs(signature[0] - zeroInputMean) <= ZERO_INPUT_TOLERANCE
                    && Math.abs(signature[1] - zeroInputStd) <= ZERO_INPUT_TOLERANCE;
            Log.i(TAG, String.format(java.util.Locale.US,
                    "%s: zero inputs give mean=%.4f std=%.4f (expected %.4f / %.4f) — %s",
                    what, signature[0], signature[1], zeroInputMean, zeroInputStd,
                    ok ? "accepted" : "REJECTED, computes the wrong thing"));
            if (ok) {
                return candidate;
            }
        } catch (Throwable t) {
            Log.w(TAG, what + ": the zero-input check did not run", t);
        }
        try {
            candidate.close();
        } catch (Throwable ignored) {
        }
        return null;
    }

    private OrtSession tryNnapiFlags(String modelPath, java.util.EnumSet<NNAPIFlags> flags) {
        OrtSession.SessionOptions opts = null;
        try {
            opts = newSessionOptions(WORKER_THREADS);
            if (flags.isEmpty()) {
                opts.addNnapi();
            } else {
                opts.addNnapi(flags);
            }
            return env.createSession(modelPath, opts);
        } catch (Throwable t) {
            Log.w(TAG, "NNAPI " + flags + " was not created: " + t.getMessage());
            closeQuietly(opts);
            return null;
        }
    }

    private OrtSession tryXnnpack(String modelPath) {
        OrtSession.SessionOptions opts = null;
        try {
            // XNNPACK keeps its own thread pool, and ORT threads on top of it only get in the way:
            // that is what its own documentation says.
            opts = newSessionOptions(/*threads=*/ 1);
            opts.addXnnpack(java.util.Collections.singletonMap(
                    "intra_op_num_threads", String.valueOf(WORKER_THREADS)));
            return env.createSession(modelPath, opts);
        } catch (Throwable t) {
            Log.w(TAG, "XNNPACK unavailable", t);
            closeQuietly(opts);
            return null;
        }
    }

    private OrtSession tryCpu(String modelPath) {
        OrtSession.SessionOptions opts = null;
        try {
            opts = newSessionOptions(WORKER_THREADS);
            return env.createSession(modelPath, opts);
        } catch (Throwable t) {
            Log.w(TAG, "CPU session was not created", t);
            closeQuietly(opts);
            return null;
        }
    }

    private static void closeQuietly(OrtSession.SessionOptions opts) {
        if (opts != null) {
            try {
                opts.close();
            } catch (Throwable ignored) {
            }
        }
    }

    private OrtSession tryNnapi(String modelPath, boolean fp16) {
        OrtSession.SessionOptions opts = null;
        try {
            opts = newSessionOptions(/*threads=*/ 2);
            if (fp16) {
                opts.addNnapi(java.util.EnumSet.of(NNAPIFlags.USE_FP16));
            } else {
                opts.addNnapi();
            }
            OrtSession s = env.createSession(modelPath, opts);
            Log.i(TAG, "NNAPI EP enabled fp16=" + fp16);
            return s;
        } catch (Throwable t) {
            Log.w(TAG, "NNAPI fp16=" + fp16 + " session failed", t);
            if (opts != null) {
                try {
                    opts.close();
                } catch (Throwable ignored) {
                }
            }
            return null;
        }
    }

    /**
     * How many threads to give the compute.
     *
     * <p>The SM8150 has eight cores, but four of them are fast: one at 2.96 GHz and three at 2.42,
     * the rest at 1.79. MEASURED: eight threads give 63.5 ms against 50.5 on four — the little cores
     * do not add, they slow things down, because a convolution waits for its slowest thread.
     */
    private static final int WORKER_THREADS = 4;

    private static OrtSession.SessionOptions newSessionOptions(int intraOpThreads) throws Exception {
        OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
        opts.setIntraOpNumThreads(Math.max(1, intraOpThreads));
        // The full optimisation set. This used to be the basic level — a workaround for extended
        // optimisations fusing Gemm with its activation into `com.microsoft.FusedGemm`, which has no
        // half-precision kernel, so the session fails at creation. The asset is fp32 now, the
        // workaround is unnecessary, and it cost noticeably: fusions are exactly what removes extra
        // passes over memory.
        //
        // Should the model turn out to be fp16 after all, `createValidatedSession` catches the refusal
        // and rebuilds the session at the basic level.
        opts.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
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
        this.warpM = ModelCalibWarp.warpMatrixDeg(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, false);
        this.warpWideM = ModelCalibWarp.warpMatrixDeg(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, true);
        this.hasPrev = false;
        this.prevFrame6 = null;
        this.prevWide6 = null;
        // The feature history belongs to the previous calibration: once the matrix shifts it
        // describes the wrong geometry. Rebuilding it over 99 frames is cheaper than driving on a
        // mixture of two calibrations.
        java.util.Arrays.fill(featuresBuffer, 0f);
        java.util.Arrays.fill(prevDesiredCurv, 0f);
        Log.i(TAG, String.format(
                "calib warp rpy_deg=(%.2f,%.2f,%.2f) K=(%.1f,%.1f,%.1f,%.1f) %dx%d",
                rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, calibW, calibH));
    }

    public synchronized void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                                      float fx, float fy, float cx, float cy) {
        setCalib(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, calibW, calibH);
    }

    /**
     * Output signature on zero inputs — the one compared against the offline reference.
     *
     * <p>The point is that this exact run is easy to repeat on a workstation: same zeros, same file,
     * `onnxruntime` from Python. There the model gives mean −1.2455 with spread 3.28. Anything else
     * here means the fault lies neither in the model nor in the frame but in the plumbing — half
     * precision, byte order or tensor shape. Without such an anchor these are indistinguishable: the
     * runner returns plausible numbers under any of those mistakes.
     */
    private float[] zeroInputSignature(OrtSession target) throws Exception {
        {
            final boolean half = isHalfPrecision(target);
            float[] zeroImgs = new float[TENSOR_C * TENSOR_H * TENSOR_W];
            long[] imgShape = new long[]{1, TENSOR_C, TENSOR_H, TENSOR_W};
            try (OnnxTensor a = tensor(half, zeroImgs, imgShape);
                 OnnxTensor b = tensor(half, zeroImgs, imgShape);
                 OnnxTensor c = tensor(half, new float[DESIRE_LEN], new long[]{1, HISTORY_LEN + 1, 8});
                 OnnxTensor d = tensor(half, new float[2], new long[]{1, 2});
                 OnnxTensor e = tensor(half, new float[2], new long[]{1, 2});
                 OnnxTensor f = tensor(half, new float[PREV_CURV_LEN], new long[]{1, PREV_CURV_LEN, 1});
                 OnnxTensor g = tensor(half, new float[HISTORY_LEN * FEATURE_LEN],
                         new long[]{1, HISTORY_LEN, FEATURE_LEN})) {
                Map<String, OnnxTensor> feeds = new HashMap<>();
                feeds.put("input_imgs", a);
                feeds.put("big_input_imgs", b);
                feeds.put("desire", c);
                feeds.put("traffic_convention", d);
                feeds.put("lateral_control_params", e);
                feeds.put("prev_desired_curv", f);
                feeds.put("features_buffer", g);
                try (OrtSession.Result result = target.run(feeds)) {
                    float[] flat = readOutput(result.get(0));
                    double sum = 0;
                    double sumsq = 0;
                    for (float v : flat) {
                        sum += v;
                        sumsq += (double) v * v;
                    }
                    double mean = sum / flat.length;
                    double std = Math.sqrt(Math.max(0, sumsq / flat.length - mean * mean));
                    return new float[]{(float) mean, (float) std};
                }
            }
        }
    }

    /**
     * Refuse to load a model of the previous generation.
     *
     * <p>The runner expects supercombo 0.9.x: two frames, feature feedback, output 6504. A 0.8.x model
     * takes four inputs and returns 6472 — a different output layout, so the pose and the lane lines
     * would be read at the wrong offsets. Nothing announces it: the numbers stay plausible.
     *
     * <p>Not a hypothetical: a file can be dropped in via {@code /sdcard/adas_models/}, and an old
     * model from earlier runs may well be sitting there.
     */
    private static void requireGeneration09x(java.util.Set<String> inputNames) {
        for (String required : new String[]{"input_imgs", "big_input_imgs", "features_buffer"}) {
            if (!inputNames.contains(required)) {
                throw new IllegalStateException(
                        "model of the wrong generation: no input " + required + ", found " + inputNames
                                + ". supercombo 0.9.x is required (9 inputs, output 6504)");
            }
        }
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


    @Override
    public void setEgoSpeed(float speedMps) {
        // Deliberately not synchronized: `run` holds the lock for the whole inference, and the CAN
        // thread must not wait on it to store one float.
        this.egoSpeedMps = Float.isFinite(speedMps) && speedMps > 0f ? speedMps : 0f;
    }

    public synchronized Result run(Bitmap frame, int frameId) throws Exception {
        return run(frame, frameId, adas.app.TimeUtil.nowMs());
    }

    /** Legacy ARGB path (CPU RGB warp). Prefer {@link #run(YuvFrame, int, long)}. */
    public synchronized Result run(Bitmap frame, int frameId, long captureTsMs) throws Exception {
        float[] m = resolveWarp(frame.getWidth(), frame.getHeight(), false);
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
        long tPrep0 = System.nanoTime();
        ModelCalibWarp.warpPair(frame, resolveWarp(frame.width, frame.height, false),
                resolveWarp(frame.width, frame.height, true), currFrame6, currWide6);
        float prepMs = (float) ((System.nanoTime() - tPrep0) / 1e6);
        return runPreparedFrame6(frameId, captureTsMs, prepMs);
    }

    private float[] resolveWarp(int frameW, int frameH, boolean wide) {
        float[] m = wide ? warpWideM : warpM;
        if (frameW != calibW || frameH != calibH) {
            float sx = frameW / (float) calibW;
            float sy = frameH / (float) calibH;
            m = ModelCalibWarp.warpMatrixDeg(
                    rollDeg, pitchDeg, yawDeg, fx * sx, fy * sy, cx * sx, cy * sy, wide);
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
            prevWide6 = currWide6.clone();
            hasPrev = true;
            return null;
        }

        // The model input is a pair of frames, previous and current, six channels each. Twice over:
        // the narrow input and the wide one.
        System.arraycopy(prevFrame6, 0, input12, 0, prevFrame6.length);
        System.arraycopy(currFrame6, 0, input12, prevFrame6.length, currFrame6.length);
        System.arraycopy(currFrame6, 0, prevFrame6, 0, currFrame6.length);
        System.arraycopy(prevWide6, 0, inputWide12, 0, prevWide6.length);
        System.arraycopy(currWide6, 0, inputWide12, prevWide6.length, currWide6.length);
        System.arraycopy(currWide6, 0, prevWide6, 0, currWide6.length);

        latParams[0] = egoSpeedMps;
        latParams[1] = 0.1f;  // actuator delay, as in flowpilot

        long[] imgShape = new long[]{1, TENSOR_C, TENSOR_H, TENSOR_W};
        try (OnnxTensor tImgs = tensor(halfPrecision, input12, imgShape);
             OnnxTensor tWide = tensor(halfPrecision, inputWide12, imgShape);
             OnnxTensor tDesire = tensor(halfPrecision, desire, new long[]{1, HISTORY_LEN + 1, 8});
             OnnxTensor tTraffic = tensor(halfPrecision, traffic, new long[]{1, 2});
             OnnxTensor tLat = tensor(halfPrecision, latParams, new long[]{1, 2});
             OnnxTensor tPrevCurv = tensor(halfPrecision, prevDesiredCurv, new long[]{1, PREV_CURV_LEN, 1});
             OnnxTensor tFeatures = tensor(halfPrecision, featuresBuffer,
                     new long[]{1, HISTORY_LEN, FEATURE_LEN})) {

            Map<String, OnnxTensor> feeds = new HashMap<>();
            feeds.put("input_imgs", tImgs);
            feeds.put("big_input_imgs", tWide);
            feeds.put("desire", tDesire);
            feeds.put("traffic_convention", tTraffic);
            feeds.put("lateral_control_params", tLat);
            feeds.put("prev_desired_curv", tPrevCurv);
            feeds.put("features_buffer", tFeatures);

            long tInfer0 = System.nanoTime();
            try (OrtSession.Result result = session.run(feeds)) {
                float inferMs = (float) ((System.nanoTime() - tInfer0) / 1e6);
                float[] flat = readOutput(result.get(0));
                advanceRecurrence(flat);
                LaneLines lanes = parseLanes(flat);
                lanes.frameId = frameId;
                long capture = captureTsMs > 0 ? captureTsMs : adas.app.TimeUtil.nowMs();
                long infer = adas.app.TimeUtil.nowMs();
                lanes.captureTimestampMs = capture;
                lanes.inferTimestampMs = infer;
                lanes.inferDurationMs = inferMs;
                lanes.prepDurationMs = prepMs;
                lanes.timestampMs = capture;
                // A copy, not a reference to the working buffer: `flat` is a field the next frame
                // overwrites, and this array travels into the bag and over ZMQ. The thneed path does
                // the same for the same reason.
                lanes.modelOut = flat.clone();
                CameraOdometry pose = CameraOdometry.parse(flat, POSE_IDX_09X);
                // The longitudinal part sits at different offsets in the 0.9.x layout and has no
                // parser yet — exactly as on the thneed path. Better to return nothing than a lead
                // read from the wrong place.
                ModelLongParse.Out modelLong = null;
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

                return new Result(lanes, pose, modelLong);
            }
        }
    }

    /**
     * A tensor in whatever precision the model expects.
     *
     * <p>Precision is a property of the file, not of the code: supercombo 0.9.7 exists both in fp16
     * and converted to fp32. Feeding the wrong one is impossible — the session refuses — so we ask the
     * session itself.
     */
    private OnnxTensor tensor(boolean half, float[] values, long[] shape) throws Exception {
        if (!half) {
            return OnnxTensor.createTensor(env, FloatBuffer.wrap(values), shape);
        }
        java.nio.ByteBuffer bytes = java.nio.ByteBuffer.allocateDirect(values.length * 2)
                .order(java.nio.ByteOrder.nativeOrder());
        java.nio.ShortBuffer halfView = bytes.asShortBuffer();
        for (float v : values) {
            halfView.put(ai.onnxruntime.platform.Fp16Conversions.floatToFp16(v));
        }
        return OnnxTensor.createTensor(env, bytes, shape, ai.onnxruntime.OnnxJavaType.FLOAT16);
    }

    /** Which precision the model expects its inputs in. */
    private static boolean isHalfPrecision(OrtSession target) {
        try {
            ai.onnxruntime.NodeInfo info = target.getInputInfo().get("input_imgs");
            return info != null && info.getInfo() instanceof ai.onnxruntime.TensorInfo
                    && ((ai.onnxruntime.TensorInfo) info.getInfo()).type
                            == ai.onnxruntime.OnnxJavaType.FLOAT16;
        } catch (Throwable t) {
            return true;
        }
    }

    /** Model output as plain floats: the session returns fp16, while everything is parsed in single precision. */
    private float[] readOutput(ai.onnxruntime.OnnxValue value) throws Exception {
        if (value instanceof OnnxTensor) {
            OnnxTensor tensor = (OnnxTensor) value;
            try {
                FloatBuffer buf = tensor.getFloatBuffer();
                if (buf != null) {
                    int n = Math.min(buf.remaining(), netOutput.length);
                    buf.get(netOutput, 0, n);
                    return netOutput;
                }
            } catch (Throwable ignored) {
                // Not every build serves fp16 through getFloatBuffer; the general path is below.
            }
        }
        return flattenOutput(value.getValue());
    }

    /**
     * Shift both histories and append the fresh frame to them.
     *
     * <p>This model's recurrence is exposed rather than internal: the network emits features in the
     * output tail and expects them back on the next frame's input. The native thneed runner does the
     * same — if these two places drift apart, so do the predictions, and silently at that.
     */
    private void advanceRecurrence(float[] flat) {
        if (flat.length < NET_OUTPUT_SIZE) {
            return;
        }
        System.arraycopy(featuresBuffer, FEATURE_LEN, featuresBuffer, 0,
                FEATURE_LEN * (HISTORY_LEN - 1));
        System.arraycopy(flat, PARSED_OUTPUT, featuresBuffer, FEATURE_LEN * (HISTORY_LEN - 1),
                FEATURE_LEN);

        System.arraycopy(prevDesiredCurv, 1, prevDesiredCurv, 0, PREV_CURV_LEN - 1);
        prevDesiredCurv[PREV_CURV_LEN - 1] = flat[DESIRED_CURV_IDX];
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

    /**
     * Parses the supercombo output.
     *
     * <p>Package-private rather than private because {@link SupercomboThneedRunner} parses the 0.9.x
     * output with the same code: the plan and lane sections of the two generations match bitwise.
     * Checked against their `driving.h` — plan element 5 x XYZ = 15 floats (our PLAN_COLS), prediction
     * 15*33*2 + 1 = 991 (PLAN_GROUP), plans 991*5 = 4955 (PLAN_END), then the same four lines of 33 YZ
     * pairs.
     *
     * <p>The generations diverge after road_edges, where 0.9.x adds wide_from_device_euler,
     * temporal_pose, road_transform and action, so only what precedes that boundary is shared and the
     * 0.9.x pose is read at its own offset.
     */
    static LaneLines parseLanes(float[] out) {
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
            int stdBase = EDGE_STDS_START + edge * 66;
            for (int i = 0; i < LaneLines.N; i++) {
                ll.edgesY[edge][i] = out[base + i * 2];
                ll.edgesZ[edge][i] = out[base + i * 2 + 1];
                ll.edgesYStd[edge][i] = (float) Math.exp(out[stdBase + i * 2]);
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

    @Override
    public void close() {
        try {
            session.close();
        } catch (Exception ignored) {
        }
    }
}
