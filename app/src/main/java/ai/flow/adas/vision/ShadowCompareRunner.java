package ai.flow.adas.vision;

import android.content.Context;
import android.util.Log;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

import ai.flow.adas.Logger;
import ai.flow.adas.Messages;
import ai.flow.adas.ProtoUtils;

/**
 * Runs two models on the same frames: one drives, the other is computed idle and written to the bag.
 * thneed is three times faster than our ONNX path, but it is a foreign 0.9.x two-camera model fed a
 * duplicated narrow frame — its speed is measured, its lane accuracy is not, and an unmeasured model
 * must not steer.
 *
 * <p><b>The shadow runs every Nth frame</b> ({@code vision.shadow_every_n}, default 3) because the two
 * models share the accelerator: measured in parallel, ONNX goes 44.7 → 61.5 ms and thneed 15.7 → 21.9,
 * both losing about 38 %. On every frame that would drop the drive to 10 Hz — the rate at which the
 * steering was jerky in the first place, so the check would spoil what it checks.
 *
 * <p>Both models are recurrent and the shadow sees frames N times further apart, so the comparison is
 * <b>biased against thneed</b>: agreement under that handicap is strong evidence, disagreement is not
 * a verdict. Compare geometry on a shared frame_id, not behaviour over time.
 */
public final class ShadowCompareRunner implements ModelRunner {

    private static final String TAG = "ShadowCompare";
    private static final String SHADOW_TOPIC = "vision/lanes_shadow";

    private final ModelRunner primary;
    private final ModelRunner shadow;
    /** The whole shadow, bag write included, so the control thread never waits on it. */
    private final ExecutorService shadowExecutor;
    private final AtomicBoolean shadowBusy = new AtomicBoolean(false);
    private final int everyN;

    private volatile long frames;
    private volatile long shadowFrames;
    private volatile long shadowSkipped;
    private volatile double primaryMs;
    private volatile double shadowMs;

    public ShadowCompareRunner(Context context) throws Exception {
        this.primary = new SupercomboOnnxRunner(context);
        this.shadow = new SupercomboThneedRunner(context);
        this.everyN = Math.max(1, ai.flow.adas.AdasConfig.shadowEveryN(context));
        this.shadowExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "ShadowModel");
            // Below the control thread: when cores are short, the measurement yields.
            t.setPriority(Thread.NORM_PRIORITY - 1);
            return t;
        });
        Log.i(TAG, "compare: " + primary.name() + " drives, " + shadow.name()
                + " shadows every " + everyN + " frames");
    }

    @Override
    public String name() {
        return "compare(" + primary.name() + " + " + shadow.name() + ")";
    }

    @Override
    public void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                         float fx, float fy, float cx, float cy, int width, int height) {
        primary.setCalib(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, width, height);
        shadow.setCalib(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, width, height);
    }

    @Override
    public void setEgoSpeed(float speedMps) {
        primary.setEgoSpeed(speedMps);
        shadow.setEgoSpeed(speedMps);
    }

    @Override
    public SupercomboOnnxRunner.Result run(YuvFrame frame, int frameId, long captureTsMs)
            throws Exception {
        if (frames % everyN == 0) {
            submitShadow(frame, frameId, captureTsMs);
        }
        SupercomboOnnxRunner.Result main = primary.run(frame, frameId, captureTsMs);
        frames++;
        if (main != null && main.lanes != null) {
            primaryMs += main.lanes.inferDurationMs;
        }
        report();
        return main;
    }

    /**
     * Handed over only when the shadow is free: a queue would accumulate stale frames and measure a
     * scene the control path no longer sees.
     *
     * <p>Two threads read the frame at once, which is safe only while {@link YuvFrame} is never
     * reused. A buffer pool (task #33) would break that and the shadow would need a copy.
     */
    private void submitShadow(YuvFrame frame, int frameId, long captureTsMs) {
        if (!shadowBusy.compareAndSet(false, true)) {
            shadowSkipped++;
            return;
        }
        try {
            shadowExecutor.execute(() -> {
                try {
                    SupercomboOnnxRunner.Result other = shadow.run(frame, frameId, captureTsMs);
                    if (other != null && other.lanes != null) {
                        shadowFrames++;
                        shadowMs += other.lanes.inferDurationMs;
                        Messages.ZMQMessage msg =
                                ProtoUtils.createLaneLinesMessage(other.lanes, true, SHADOW_TOPIC);
                        if (msg != null) {
                            Logger.getInstance().logZMQMessage(msg);
                        }
                    }
                } catch (Throwable t) {
                    // The shadow must never take the drive down; it is not the one steering.
                    Log.e(TAG, "shadow model failed on frame " + frameId, t);
                } finally {
                    shadowBusy.set(false);
                }
            });
        } catch (Throwable t) {
            shadowBusy.set(false);
            Log.e(TAG, "shadow not submitted", t);
        }
    }

    /** Summary against the known solo timings, so it is visible on the drive whether the shadow costs. */
    private void report() {
        long n = frames;
        if (n % 100 != 0) {
            return;
        }
        long sn = shadowFrames;
        Log.i(TAG, String.format(
                "%d frames: %s %.1f ms (solo 44.7, paired 61.5), %s %.1f ms over %d frames "
                        + "(solo 15.7, paired 21.9), shadow missed %d",
                n, primary.name(), primaryMs / n,
                shadow.name(), sn > 0 ? shadowMs / sn : 0.0, sn, shadowSkipped));
    }

    @Override
    public void close() {
        shadowExecutor.shutdown();
        primary.close();
        shadow.close();
    }
}
