package ai.flow.adas.vision;

import android.content.Context;
import android.util.Log;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;

/**
 * Runs the flowpilot 0.9.x model on the phone GPU through thneed: 15.9 ms per frame on this device
 * against 44.7 for ours through ONNX, which is what lets the pipeline hold the full 30 Hz camera rate.
 * Background in docs/VISION_RATE.md.
 *
 * <p>thneed is not a runtime but a serialized sequence of OpenCL kernels with their arguments, stored
 * as compiled GPU binaries. The file is therefore tied to the GPU and driver that produced it, so our
 * own network cannot be substituted here without compiling it on the device.
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
    }

    private static byte[] readAsset(Context context, String name) throws Exception {
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
        ModelCalibWarp.warpYuvToFrame6(frame, warpM, curr6);
        ModelCalibWarp.warpYuvToFrame6(frame, warpWideM, currWide6);
        float prepMs = (float) ((System.nanoTime() - tPrep0) / 1e6);

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
        input[2 * IMG_LEN + DESIRE_LEN + 1] = 0.1f;  // задержка актюатора, как у flowpilot

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
        lanes.frameId = frameId;
        lanes.timestampMs = captureTsMs;
        lanes.captureTimestampMs = captureTsMs;
        lanes.inferTimestampMs = ai.flow.adas.TimeUtil.nowMs();
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

    @Override
    public void close() {
        // The native side owns its GPU buffers for the process lifetime; re-initialisation is untested.
    }
}
