package adas.app.vision;

import adas.app.AdasConfig;
import adas.app.TimeUtil;
import android.content.Context;
import android.util.Log;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;

/**
 * Runs supercombo 0.9.7 on the phone GPU through thneed: 17 ms of inference and 24 ms per frame on
 * this device against 50 and 54 through ONNX, which is what lets the pipeline hold the full 30 Hz
 * camera rate. Background in docs/VISION_RATE.md and docs/THNEED.md.
 *
 * <p>thneed is not a runtime but a serialized run of the network on the GPU: kernel sources, their
 * launch order, and the buffers they read. Upstream openpilot replays such a recording at the Adreno
 * ioctl layer; this loader does not — it creates ordinary OpenCL buffers and enqueues ordinary
 * kernels, so an Adreno is not required, only an OpenCL the app can reach. The file ships kernel
 * <em>sources</em>, compiled by the driver of whichever GPU is going to run them.
 *
 * <p>The same network is in assets/supercombo.onnx, and the thneed is generated from it by
 * scripts/tools/thneed_from_onnx.py, byte-for-byte reproducibly.
 *
 * <p>The output is parsed by {@link SupercomboOnnxRunner#parseLanes} because the plan and lane sections
 * of the two generations match bitwise: plans 15*33*2+1 = 991 per hypothesis, 991*5 = 4955, then the
 * same four lines of 33 YZ pairs — our PLAN_COLS, PLAN_GROUP and PLAN_END.
 *
 * <p>Not parsed here: the longitudinal part (leads, meta). It exists in the output at different
 * offsets, so {@code modelLong} is null and no longitudinal plan is published in this mode.
 */
public final class SupercomboThneedRunner implements ModelRunner {

    @Override
    public String name() {
        return "thneed";
    }


    private static final String TAG = "ThneedRunner";

    /** Flat input: two 12x128x256 images, desire 100x8, then vEgo and the actuator delay. */
    private static final int IMG_LEN = SupercomboOnnxRunner.TENSOR_C
            * SupercomboOnnxRunner.TENSOR_H * SupercomboOnnxRunner.TENSOR_W;
    private static final int DESIRE_LEN = 3200 / 4;
    private static final int LAT_PARAMS_LEN = 2;
    private static final int INPUT_LEN = 2 * IMG_LEN + DESIRE_LEN + LAT_PARAMS_LEN;

    /** 5992 parsed values + 512 features the model feeds back to itself. */
    private static final int NET_OUTPUT_SIZE = 6504;

    /**
     * Camera pose offset in the 0.9.x output, summed from their driving.h: plans 4955 + lane_lines 536
     * + road_edges 264 + leads 105 + meta 88. Cross-check: pose 12 + wide_from_device_euler 6 +
     * temporal_pose 12 + road_transform 12 + action 1 = 5991, plus PAD_SIZE 1 = 5992, which is where
     * the JNI takes the features from.
     *
     * <p>The length heuristic used for our model gives 5980 here, because 0.9.x has more fields after
     * the pose. Pose feeds the online calibration whose warp feeds the model, so a wrong offset would
     * quietly corrupt the lane output too.
     */
    private static final int POSE_IDX = 5948;

    private static final String ASSET = "supercombo.thneed";

    private static boolean libraryLoaded;
    private static boolean libraryTried;

    private static native boolean nativeInit(byte[] modelData, int outLen);

    /** @return execution time in ms, or negative on failure. */
    private static native float nativeExecute(float[] input, float[] output);

    private static native boolean nativeSave(String path, boolean binaries);

    private static native String nativeClInfo();

    /**
     * What OpenCL can do on this device, as JSON.
     *
     * <p>The question "will it run on another phone" reduces to four checkable things: is libOpenCL
     * reachable by the app, is `cl_khr_fp16` present, can the device make images over buffers, and is
     * the required row-pitch alignment no coarser than what the file assumes. All of it is handed out
     * here rather than logged, so `tools/model_device_probe.py` can answer for a new phone without
     * anyone running anything by hand.
     *
     * <p>Returns {@code {"opencl": false, ...}} when the runner library does not load: on such a
     * device the ONNX path is what remains.
     */
    public static String openClInfo() {
        if (!available()) {
            return "{\"opencl\": false, \"reason\": \"libthneedrunner did not load\"}";
        }
        try {
            return nativeClInfo();
        } catch (Throwable t) {
            return "{\"opencl\": false, \"reason\": \"nativeClInfo unavailable\"}";
        }
    }

    /**
     * Save the current model as compiled by this GPU.
     *
     * <p>The point is that the Adreno compiler on the phone knows its own hardware, while the machine
     * that prepared the file does not. Call after at least one run: before that there is nothing to
     * write.
     *
     * @param path where to write
     * @param binaries true — binaries for this device, false — kernel sources
     */
    public static boolean saveCompiled(String path, boolean binaries) {
        return nativeSave(path, binaries);
    }

    /**
     * Output signature on zero inputs, taken on a workstation when the file was built
     * (`scripts/tools/thneed_from_onnx.py` writes the reference next to the thneed).
     *
     * <p>Serves as acceptance on a new phone. The file carries kernel sources, built by the driver of
     * whichever device is going to run the model: whether it loaded is obvious at once, whether it
     * computes the same thing is not. Differences in image row-pitch handling or in driver rounding
     * yield plausible but foreign numbers, and a road frame cannot tell them apart.
     */
    private static final float ZERO_INPUT_MEAN = -1.2498f;
    private static final float ZERO_INPUT_STD = 3.2745f;

    /** The same signature taken from the config when set there: it belongs to the model file. */
    private float zeroInputMean = ZERO_INPUT_MEAN;
    private float zeroInputStd = ZERO_INPUT_STD;
    /** A loose tolerance: we catch a foreign answer, not a difference in the last digit. */
    private static final float ZERO_INPUT_TOLERANCE = 0.5f;

    private final float[] input = new float[INPUT_LEN];
    private final float[] output = new float[NET_OUTPUT_SIZE];
    private final float[] curr6 = new float[SupercomboOnnxRunner.CH_PER_FRAME
            * SupercomboOnnxRunner.TENSOR_H * SupercomboOnnxRunner.TENSOR_W];
    private final float[] prev6 = new float[curr6.length];
    private final float[] currWide6 = new float[curr6.length];
    private final float[] prevWide6 = new float[curr6.length];
    private boolean hasPrev;

    private float rollDeg, pitchDeg, yawDeg;
    private float fx, fy, cx, cy;
    private float[] warpM, warpWideM;

    /**
     * Below this, input sharpness means there is nothing to look at.
     *
     * <p>The healthy drives of 08-13 gave 369-942, the defocused drive of 08-16 gave 9.9-14.9. The
     * threshold sits in the middle on a log scale; no intermediate values occurred in the data.
     */
    private static final float FOCUS_MIN_SCORE = 60f;

    private long focusFrames;
    private long frames;
    private double inferSumMs;
    /** Written by the ZMQ thread, read by the inference thread. */
    private volatile float egoSpeedMps;

    /** Library load is separate from the constructor and never throws: the app must stay on ONNX. */
    public static synchronized boolean available() {
        if (!libraryTried) {
            libraryTried = true;
            try {
                System.loadLibrary("thneedrunner");
                libraryLoaded = true;
            } catch (Throwable t) {
                Log.e(TAG, "libthneedrunner not loaded — staying on ONNX", t);
                libraryLoaded = false;
            }
        }
        return libraryLoaded;
    }

    public SupercomboThneedRunner(Context context) throws Exception {
        if (!available()) {
            throw new IllegalStateException("thneed runner unavailable");
        }
        byte[] model = readAsset(context, ASSET);
        Log.i(TAG, "loading " + ASSET + ", " + (model.length / (1024 * 1024)) + " MB");
        long t0 = System.nanoTime();
        if (!nativeInit(model, NET_OUTPUT_SIZE)) {
            throw new IllegalStateException("nativeInit failed");
        }
        Log.i(TAG, String.format("thneed ready in %.0f ms", (System.nanoTime() - t0) / 1e6));
        final float[] signature = AdasConfig.zeroInputSignature(context, "thneed");
        if (signature != null) {
            zeroInputMean = signature[0];
            zeroInputStd = signature[1];
        }
        requireSaneOnZeros();
    }

    /**
     * Model lookup: the external file first, the asset second.
     *
     * <p>Same as in the ONNX runner and for the same reason: testing a new thneed by rebuilding the
     * APK every iteration means 300 MB and several minutes per attempt. The external path lets one
     * push a file over adb and measure straight away.
     */
    private static byte[] readAsset(Context context, String name) throws Exception {
        java.io.File external = new java.io.File("/sdcard/adas_models/" + name);
        if (external.isFile() && external.length() > 0) {
            Log.i(TAG, "loading " + external.getAbsolutePath() + " (" + external.length() + " bytes)");
            try (InputStream in = new java.io.FileInputStream(external)) {
                ByteArrayOutputStream out = new ByteArrayOutputStream(1 << 20);
                byte[] buf = new byte[1 << 16];
                int n;
                while ((n = in.read(buf)) > 0) {
                    out.write(buf, 0, n);
                }
                return out.toByteArray();
            }
        }
        try (InputStream in = context.getAssets().open(name)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream(1 << 20);
            byte[] buf = new byte[1 << 16];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
            }
            return out.toByteArray();
        }
    }

    @Override
    public synchronized void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                                      float fx, float fy, float cx, float cy, int width, int height) {
        this.rollDeg = rollDeg;
        this.pitchDeg = pitchDeg;
        this.yawDeg = yawDeg;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.warpM = ModelCalibWarp.warpMatrixDeg(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, false);
        this.warpWideM = ModelCalibWarp.warpMatrixDeg(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, true);
    }

    /**
     * Run the model on zero inputs and compare against the reference taken when the file was built.
     *
     * <p>If it fails we throw: the app falls back to the ONNX path, which checks itself the same way.
     * Driving on lane lines from a model computing who-knows-what is worse than driving slower.
     *
     * <p>A side effect worth knowing about: `nativeExecute` shifts the feature and desired-curvature
     * history at the end, so after the check one zero frame remains inside it. It washes out over 99
     * frames — the same first seconds the model spends warming up anyway.
     */
    private void requireSaneOnZeros() {
        java.util.Arrays.fill(input, 0f);
        float ms = nativeExecute(input, output);
        if (ms < 0) {
            throw new IllegalStateException("thneed failed to run on zero inputs");
        }
        double sum = 0;
        double sumsq = 0;
        for (float v : output) {
            sum += v;
            sumsq += (double) v * v;
        }
        final double mean = sum / output.length;
        final double std = Math.sqrt(Math.max(0, sumsq / output.length - mean * mean));
        final boolean ok = Math.abs(mean - zeroInputMean) <= ZERO_INPUT_TOLERANCE
                && Math.abs(std - zeroInputStd) <= ZERO_INPUT_TOLERANCE;
        Log.i(TAG, String.format(java.util.Locale.US,
                "zero-input check: mean=%.4f std=%.4f (expected %.4f / %.4f) — %s",
                mean, std, zeroInputMean, zeroInputStd, ok ? "accepted" : "REJECTED"));
        if (!ok) {
            throw new IllegalStateException(String.format(java.util.Locale.US,
                    "thneed computes the wrong thing on this GPU: zero inputs give mean=%.4f std=%.4f instead of %.4f / %.4f",
                    mean, std, zeroInputMean, zeroInputStd));
        }
    }

    @Override
    public void setEgoSpeed(float speedMps) {
        // Deliberately not synchronized: `run` holds the lock for the whole inference, and the CAN
        // thread must not wait on it to store one float.
        this.egoSpeedMps = Float.isFinite(speedMps) && speedMps > 0f ? speedMps : 0f;
    }

    @Override
    public synchronized SupercomboOnnxRunner.Result run(YuvFrame frame, int frameId, long captureTsMs) {
        if (warpM == null) {
            return null;
        }
        // Two warps per frame, narrow and wide: the 0.9.x model takes two camera inputs and we have
        // one camera, so the same frame goes through both matrices. This doubles preparation time.
        long tPrep0 = System.nanoTime();
        ModelCalibWarp.warpPair(frame, warpM, warpWideM, curr6, currWide6);
        float prepMs = (float) ((System.nanoTime() - tPrep0) / 1e6);
        reportFocus(ModelCalibWarp.lastFocusScore());

        if (!hasPrev) {
            System.arraycopy(curr6, 0, prev6, 0, curr6.length);
            System.arraycopy(currWide6, 0, prevWide6, 0, currWide6.length);
            hasPrev = true;
            return null;
        }

        // Model input is a pair of frames, previous and current, six channels each.
        System.arraycopy(prev6, 0, input, 0, prev6.length);
        System.arraycopy(curr6, 0, input, prev6.length, curr6.length);
        System.arraycopy(prevWide6, 0, input, IMG_LEN, prevWide6.length);
        System.arraycopy(currWide6, 0, input, IMG_LEN + prevWide6.length, currWide6.length);
        System.arraycopy(curr6, 0, prev6, 0, curr6.length);
        System.arraycopy(currWide6, 0, prevWide6, 0, currWide6.length);

        // desire stays zero: there is no lane-change planner here and the model runs without one.
        input[2 * IMG_LEN + DESIRE_LEN] = egoSpeedMps;
        input[2 * IMG_LEN + DESIRE_LEN + 1] = 0.1f;  // actuator delay, as in flowpilot

        float ms = nativeExecute(input, output);
        if (ms < 0) {
            Log.e(TAG, "nativeExecute failed");
            return null;
        }
        frames++;
        inferSumMs += ms;
        if (frames % 100 == 0) {
            Log.i(TAG, String.format("thneed inference: %.1f ms mean over %d frames (last %.1f)",
                    inferSumMs / frames, frames, ms));
        }

        LaneLines lanes = SupercomboOnnxRunner.parseLanes(output);
        // The same vector as on the ONNX path: without it a bag cannot be re-parsed offline and a
        // measurement on a new phone has nothing to compare against — the runner ran, but whether
        // the numbers are the right ones is unanswerable.
        lanes.modelOut = output.clone();
        lanes.frameId = frameId;
        lanes.timestampMs = captureTsMs;
        lanes.captureTimestampMs = captureTsMs;
        lanes.inferTimestampMs = adas.app.TimeUtil.nowMs();
        lanes.inferDurationMs = ms;
        lanes.prepDurationMs = prepMs;
        // Pose feeds the camera calibration, whose warp feeds the model. The struct is the same across
        // generations; only its offset differs.
        CameraOdometry pose = CameraOdometry.parse(output, POSE_IDX);
        // Kept alongside the wheel speed: the pose scale is still unverified against CAN (task #37).
        if (frames % 100 == 0) {
            Log.i(TAG, String.format(
                    "pose: v=[%.2f %.2f %.2f] m/s  rot=[%.3f %.3f %.3f] rad/s, wheels %.2f m/s",
                    pose.trans[0], pose.trans[1], pose.trans[2],
                    pose.rot[0], pose.rot[1], pose.rot[2], egoSpeedMps));
        }
        return new SupercomboOnnxRunner.Result(lanes, pose, null);
    }

    /**
     * Say out loud when the model is looking at a blur.
     *
     * <p>The threshold sits with a wide margin between the healthy drives (369-942) and the defocused
     * one (9.9-14.9). There are no intermediate values in the data, so it is hard to get wrong; the
     * quantity is dimensionless and is only ever compared against itself.
     *
     * <p>Once every hundred frames rather than every frame: the point is for this to be visible in a
     * drive's log from the start, not to flood it.
     */
    private void reportFocus(float score) {
        if (focusFrames++ % 100 != 0) {
            return;
        }
        if (score < FOCUS_MIN_SCORE) {
            Log.w(TAG, String.format(java.util.Locale.US,
                    "OUT OF FOCUS: input sharpness %.1f against a floor of %.0f — no lane lines in such a "
                            + "frame, check the lens and the glass",
                    score, FOCUS_MIN_SCORE));
        } else {
            Log.i(TAG, String.format(java.util.Locale.US, "input sharpness %.1f", score));
        }
    }

    @Override
    public void close() {
        // The native side owns its GPU buffers for the process lifetime; re-initialisation is untested.
    }
}
