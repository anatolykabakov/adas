package ai.flow.adas.vision;

import android.content.Context;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Process;
import android.util.Log;

import ai.flow.adas.Logger;
import ai.flow.adas.Messages;
import ai.flow.adas.ProtoUtils;
import ai.flow.adas.ZMQBridgeService;

/**
 * Low-frequency traffic YOLO on a dedicated {@code TrafficYoloInfer} thread.
 * Never shares {@code SupercomboInfer}. Camera path should call
 * {@link #wantsFrame()} before {@link YuvFrame#duplicate()} so the control
 * camera callback is not charged for YOLO copies at full FPS.
 */
public final class TrafficVisionPipeline {
    private static final String TAG = "TrafficVision";
    /** Min gap between infer starts (ms). ~3 Hz. */
    public static final long PERIOD_MS = 333;

    private final HandlerThread thread;
    private final Handler handler;
    private final TrafficYoloRunner runner;
    private final LaneOverlayView overlay;

    private final Object lock = new Object();
    private boolean busy;
    private YuvFrame pending;
    private long pendingTs;
    private long lastStartMs;
    private int frameId;

    public TrafficVisionPipeline(Context context, LaneOverlayView overlay) throws Exception {
        this.overlay = overlay;
        boolean signs = ai.flow.adas.AdasConfig.visionTrafficSignsEnabled(context);
        boolean lights = ai.flow.adas.AdasConfig.visionTrafficLightsEnabled(context);
        this.runner = new TrafficYoloRunner(context.getApplicationContext(), "auto", signs, lights);
        thread = new HandlerThread("TrafficYoloInfer", Process.THREAD_PRIORITY_BACKGROUND);
        thread.start();
        handler = new Handler(thread.getLooper());
        Log.i(TAG, "ready period=" + PERIOD_MS
                + "ms thread=TrafficYoloInfer signs=" + signs + " lights=" + lights
                + " (isolated from SupercomboInfer)");
    }

    /**
     * True when the next camera frame should be copied and submitted.
     * Cheap; call on the camera callback before {@link YuvFrame#duplicate()}.
     */
    public boolean wantsFrame() {
        synchronized (lock) {
            if (busy) {
                return false;
            }
            return ai.flow.adas.TimeUtil.nowMs() - lastStartMs >= PERIOD_MS;
        }
    }

    /**
     * Takes ownership of {@code frame}. Prefer calling only after {@link #wantsFrame()}
     * (or when this is the sole YUV consumer).
     */
    public void submitYuv(YuvFrame frame, long captureTsMs) {
        if (frame == null) {
            return;
        }
        final long ts = captureTsMs > 0 ? captureTsMs : ai.flow.adas.TimeUtil.nowMs();
        synchronized (lock) {
            pending = frame;
            pendingTs = ts;
            if (busy) {
                return;
            }
            long now = ai.flow.adas.TimeUtil.nowMs();
            if (now - lastStartMs < PERIOD_MS) {
                return;
            }
            busy = true;
            lastStartMs = now;
        }
        handler.post(this::drain);
    }

    private void drain() {
        YuvFrame frame;
        long ts;
        int id;
        synchronized (lock) {
            frame = pending;
            ts = pendingTs;
            pending = null;
            if (frame == null) {
                busy = false;
                return;
            }
            id = frameId++;
        }
        try {
            Process.setThreadPriority(Process.THREAD_PRIORITY_BACKGROUND);
            TrafficYoloRunner.Result res = runner.run(frame);
            publish(res, id, ts);
            Log.i(TAG, String.format(java.util.Locale.US,
                    "infer total=%dms prep=%d ort=%d decode=%d ocr=%d dets=%d ep=%s",
                    res.inferMs, res.prepMs, res.ortMs, res.decodeMs, res.ocrMs,
                    res.dets != null ? res.dets.size() : 0, res.ep));
        } catch (Exception e) {
            Log.e(TAG, "infer failed", e);
        } finally {
            synchronized (lock) {
                busy = false;
                // Next sample comes from camera after PERIOD_MS via wantsFrame().
            }
        }
    }

    private void publish(TrafficYoloRunner.Result res, int id, long captureTsMs) {
        if (res == null) {
            return;
        }
        long e2e = 0;
        if (captureTsMs > 0) {
            e2e = Math.max(0, ai.flow.adas.TimeUtil.nowMs() - captureTsMs);
        }
        if (overlay != null) {
            overlay.setTrafficDets(res.dets);
            overlay.setTrafficLatency(res.prepMs, res.ortMs, res.decodeMs, res.ocrMs, res.inferMs, (int) e2e);
        }
        Messages.ZMQMessage msg = ProtoUtils.createTrafficDetectionsMessage(res, id, captureTsMs);
        ZMQBridgeService.publishToNative(msg);
        if (Logger.getInstance().isRunning()) {
            Logger.getInstance().logZMQMessage(msg);
        }
    }

    public void close() {
        handler.removeCallbacksAndMessages(null);
        thread.quitSafely();
        runner.close();
    }
}
