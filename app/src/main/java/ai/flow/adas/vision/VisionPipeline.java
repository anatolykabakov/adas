package ai.flow.adas.vision;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import ai.flow.adas.Logger;
import ai.flow.adas.Messages;
import ai.flow.adas.ProtoUtils;
import ai.flow.adas.ZMQBridgeService;

/**
 * Vision infer is single-flight on {@code SupercomboInfer}, but camera keeps a
 * one-slot <b>latest</b> frame. While busy, new frames overwrite the slot; when
 * infer finishes, the latest pending frame starts immediately (no wait for the
 * next ImageReader tick).
 */
public class VisionPipeline {
    private static final String TAG = "VisionPipeline";

    private final HandlerThread thread;
    private final Handler handler;
    private final ExecutorService bagExecutor;
    /** The model is an implementation detail behind {@link ModelRunner}. */
    private final ModelRunner runner;
    private final LaneOverlayView overlay;
    private final boolean publishPose;

    private final Object lock = new Object();
    private boolean busy;
    private YuvFrame pendingYuv;
    private long pendingCaptureTs;
    /** When {@link #submitYuv} received the pending frame — camera to app delivery. */
    private long pendingSubmitTs;
    /**
     * Captures overwritten in the 1-slot buffer since the last one that got processed. This is the
     * number that separates the two explanations for a slow loop: frames arriving late (zero drops,
     * long delivery) against inference not keeping up (drops every cycle).
     */
    private int pendingDropped;
    private int frameId;
    /** Set by {@link #close}, read by the inference thread. */
    private volatile boolean closed;

    public VisionPipeline(Context context, LaneOverlayView overlay) throws Exception {
        this(context, overlay, true);
    }

    public VisionPipeline(Context context, LaneOverlayView overlay, boolean cameraCalib)
            throws Exception {
        this(context, overlay, cameraCalib, ai.flow.adas.AdasConfig.modelRunner(context));
    }

    /**
     * The runner is passed explicitly rather than read from the file, so the settings switch changes the
     * model live without saving first — the same way the pp/mpc/fp controller radio behaves.
     */
    public VisionPipeline(Context context, LaneOverlayView overlay, boolean cameraCalib,
                          String runnerName) throws Exception {
        this.overlay = overlay;
        this.publishPose = cameraCalib;
        this.runner = createRunner(context.getApplicationContext(), runnerName);
        android.util.Log.i("VisionPipeline", "model runner: " + runner.name());
        thread = new HandlerThread("SupercomboInfer");
        thread.start();
        handler = new Handler(thread.getLooper());
        bagExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "SupercomboBag");
            t.setPriority(Thread.NORM_PRIORITY - 1);
            return t;
        });
    }

    /** Falls back to ONNX: thneed is a separate .so on the GPU and must never take the app down. */
    private static ModelRunner createRunner(Context context, String choice) throws Exception {
        if ("thneed".equals(choice)) {
            try {
                return new SupercomboThneedRunner(context);
            } catch (Throwable e) {
                android.util.Log.e("VisionPipeline", "thneed unavailable — falling back to ONNX", e);
            }
        } else if ("compare".equals(choice)) {
            try {
                return new ShadowCompareRunner(context);
            } catch (Throwable e) {
                android.util.Log.e("VisionPipeline", "shadow compare unavailable — falling back to ONNX", e);
            }
        }
        return new SupercomboOnnxRunner(context);
    }

    public void submitBitmap(Bitmap bitmap) {
        submitBitmap(bitmap, ai.flow.adas.TimeUtil.nowMs());
    }

    /** Legacy ARGB path — still drop-if-busy (prefer {@link #submitYuv}). */
    public void submitBitmap(Bitmap bitmap, long captureTsMs) {
        if (bitmap == null) {
            return;
        }
        synchronized (lock) {
            if (busy) {
                return;
            }
            busy = true;
        }
        final Bitmap copy = bitmap.copy(Bitmap.Config.ARGB_8888, false);
        final int id = frameId++;
        final long captureTs = captureTsMs > 0 ? captureTsMs : ai.flow.adas.TimeUtil.nowMs();
        handler.post(() -> {
            SupercomboOnnxRunner.Result res = null;
            try {
                res = runner.run(copy, id, captureTs);
                publishControl(res, id);
            } catch (Exception e) {
                Log.e(TAG, "infer failed", e);
            } finally {
                copy.recycle();
                synchronized (lock) {
                    busy = false;
                }
            }
            submitBag(res, id);
            handler.post(this::drainYuvLatest);
        });
    }

    /**
     * Always accepts the frame into a 1-slot latest buffer. Starts infer if idle;
     * otherwise overwrites pending so the next drain runs the newest capture.
     */
    public void submitYuv(YuvFrame frame, long captureTsMs) {
        if (frame == null) {
            return;
        }
        final long captureTs = captureTsMs > 0 ? captureTsMs : ai.flow.adas.TimeUtil.nowMs();
        final long submitTs = ai.flow.adas.TimeUtil.nowMs();
        boolean start;
        synchronized (lock) {
            if (pendingYuv != null) {
                pendingDropped++;  // overwriting an unprocessed capture
            }
            pendingYuv = frame;
            pendingCaptureTs = captureTs;
            pendingSubmitTs = submitTs;
            if (busy) {
                return;
            }
            busy = true;
            start = true;
        }
        if (start) {
            handler.post(this::drainYuvLatest);
        }
    }

    /** Run until pending latest is empty. Must run on {@link #handler}. */
    private void drainYuvLatest() {
        while (true) {
            final YuvFrame frame;
            final long captureTs;
            final long submitTs;
            final int dropped;
            synchronized (lock) {
                frame = pendingYuv;
                captureTs = pendingCaptureTs;
                submitTs = pendingSubmitTs;
                dropped = pendingDropped;
                pendingYuv = null;
                pendingDropped = 0;
                if (frame == null) {
                    busy = false;
                    return;
                }
                // Stay busy across back-to-back frames.
                busy = true;
            }

            final long pickupTs = ai.flow.adas.TimeUtil.nowMs();
            final int id = frameId++;
            SupercomboOnnxRunner.Result res = null;
            try {
                res = runner.run(frame, id, captureTs);
                if (res != null && res.lanes != null) {
                    res.lanes.submitTimestampMs = submitTs;
                    res.lanes.pickupTimestampMs = pickupTs;
                    res.lanes.framesDropped = dropped;
                }
                publishControl(res, id);
            } catch (Exception e) {
                Log.e(TAG, "infer failed", e);
            }
            // Bag off the infer thread so a pending latest can start immediately.
            submitBag(res, id);
            // Loop: pick up a newer frame that arrived during this run.
        }
    }

    /**
     * Bag writes off the inference thread, and out of the way of the shutdown race: {@link #close}
     * stops {@link #bagExecutor} while a frame may still be in flight, which would otherwise throw
     * RejectedExecutionException on the inference thread. Live model switching makes that routine.
     */
    private void submitBag(SupercomboOnnxRunner.Result res, int id) {
        if (closed) {
            return;
        }
        try {
            bagExecutor.execute(() -> {
                try {
                    publishBag(res, id);
                } catch (Exception e) {
                    Log.e(TAG, "bag log failed", e);
                }
            });
        } catch (java.util.concurrent.RejectedExecutionException e) {
            // Closed between the check and the submit; the last frame is not worth keeping.
        }
    }

    private void publishControl(SupercomboOnnxRunner.Result res, int id) {
        if (res == null || res.lanes == null) {
            return;
        }
        LaneLines lanes = res.lanes;
        overlay.setLanes(lanes);
        if (res.modelLong != null) {
            overlay.setModelLong(res.modelLong);
        }
        Messages.ZMQMessage ctrlMsg = ProtoUtils.createLaneLinesMessage(lanes, false);
        if (ctrlMsg != null) {
            ZMQBridgeService.publishToNative(ctrlMsg);
        }
        if (publishPose && res.pose != null && res.pose.valid) {
            Messages.ZMQMessage poseMsg =
                    ProtoUtils.createCameraOdometryMessage(lanes.timestampMs, id, res.pose);
            if (poseMsg != null) {
                ZMQBridgeService.publishToNative(poseMsg);
            }
        }
        if (res.modelLong != null && res.modelLong.ok) {
            Messages.ZMQMessage longMsg =
                    ProtoUtils.createModelLongPlanMessage(lanes.timestampMs, id, res.modelLong, res.pose);
            if (longMsg != null) {
                ZMQBridgeService.publishToNative(longMsg);
            }
        }
    }

    private void publishBag(SupercomboOnnxRunner.Result res, int id) {
        if (res == null || res.lanes == null) {
            return;
        }
        LaneLines lanes = res.lanes;
        Messages.ZMQMessage bagMsg = ProtoUtils.createLaneLinesMessage(lanes, true);
        if (bagMsg != null) {
            Logger.getInstance().logZMQMessage(bagMsg);
        }
        if (publishPose && res.pose != null && res.pose.valid) {
            Messages.ZMQMessage poseMsg =
                    ProtoUtils.createCameraOdometryMessage(lanes.timestampMs, id, res.pose);
            if (poseMsg != null) {
                Logger.getInstance().logZMQMessage(poseMsg);
            }
        }
        if (res.modelLong != null && res.modelLong.ok) {
            Messages.ZMQMessage longMsg =
                    ProtoUtils.createModelLongPlanMessage(lanes.timestampMs, id, res.modelLong, res.pose);
            if (longMsg != null) {
                Logger.getInstance().logZMQMessage(longMsg);
            }
        }
    }

    public void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                         float fx, float fy, float cx, float cy,
                         int width, int height) {
        runner.setCalib(rollDeg, pitchDeg, yawDeg, fx, fy, cx, cy, width, height);
    }

    /** CAN speed, an input of the 0.9.x model. Arrives at 100 Hz; the runner takes the latest value. */
    public void setEgoSpeed(float speedMps) {
        runner.setEgoSpeed(speedMps);
    }

    public void close() {
        closed = true;
        synchronized (lock) {
            pendingYuv = null;
            busy = true; // stop accepting
        }
        handler.post(runner::close);
        thread.quitSafely();
        bagExecutor.shutdown();
    }
}
