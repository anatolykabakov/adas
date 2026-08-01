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
    private final SupercomboOnnxRunner runner;
    private final LaneOverlayView overlay;
    private final boolean publishPose;

    private final Object lock = new Object();
    private boolean busy;
    private YuvFrame pendingYuv;
    private long pendingCaptureTs;
    private int frameId;

    public VisionPipeline(Context context, LaneOverlayView overlay) throws Exception {
        this(context, overlay, true);
    }

    public VisionPipeline(Context context, LaneOverlayView overlay, boolean cameraCalib)
            throws Exception {
        this.overlay = overlay;
        this.publishPose = cameraCalib;
        this.runner = new SupercomboOnnxRunner(context.getApplicationContext());
        thread = new HandlerThread("SupercomboInfer");
        thread.start();
        handler = new Handler(thread.getLooper());
        bagExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "SupercomboBag");
            t.setPriority(Thread.NORM_PRIORITY - 1);
            return t;
        });
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
            final SupercomboOnnxRunner.Result bagRes = res;
            final int bagId = id;
            bagExecutor.execute(() -> {
                try {
                    publishBag(bagRes, bagId);
                } catch (Exception e) {
                    Log.e(TAG, "bag log failed", e);
                }
            });
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
        boolean start;
        synchronized (lock) {
            pendingYuv = frame;
            pendingCaptureTs = captureTs;
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
            synchronized (lock) {
                frame = pendingYuv;
                captureTs = pendingCaptureTs;
                pendingYuv = null;
                if (frame == null) {
                    busy = false;
                    return;
                }
                // Stay busy across back-to-back frames.
                busy = true;
            }

            final int id = frameId++;
            SupercomboOnnxRunner.Result res = null;
            try {
                res = runner.run(frame, id, captureTs);
                publishControl(res, id);
            } catch (Exception e) {
                Log.e(TAG, "infer failed", e);
            }
            // Bag off the infer thread so a pending latest can start immediately.
            final SupercomboOnnxRunner.Result bagRes = res;
            final int bagId = id;
            bagExecutor.execute(() -> {
                try {
                    publishBag(bagRes, bagId);
                } catch (Exception e) {
                    Log.e(TAG, "bag log failed", e);
                }
            });
            // Loop: pick up a newer frame that arrived during this run.
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

    public void close() {
        synchronized (lock) {
            pendingYuv = null;
            busy = true; // stop accepting
        }
        handler.post(runner::close);
        thread.quitSafely();
        bagExecutor.shutdown();
    }
}
