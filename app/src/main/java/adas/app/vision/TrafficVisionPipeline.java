package adas.app.vision;

import android.content.Context;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Process;
import android.util.Log;

import adas.app.Logger;
import adas.app.bridge.ProtoUtils;
import adas.app.bridge.ZMQBridgeService;
import adas.proto.Messages;

/** Traffic-light detection on its own background thread, throttled to protect the driving loop. */
public final class TrafficVisionPipeline {
    private static final String TAG = "TrafficVision";
    /** Detector period [ms]; ~3 Hz is the measured safe duty next to supercombo. */
    public static final long PERIOD_MS = 333;

    /** Back off when the vision loop falls below this share of its target rate. */
    private static final float VISION_HEALTH_FRAC = 0.75f;
    private static final int UNHEALTHY_TO_BACKOFF = 3;
    private static final int HEALTHY_TO_RESTORE = 20;
    private static final long MAX_BACKOFF = 4;

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

    /** Period multiplier while the vision loop is unhealthy; 1 = normal. */
    private volatile long backoff = 1;
    private int unhealthy;
    private int healthy;

    /** Vision rate, reported by {@link VisionPipeline}; the baseline is the camera's target rate. */
    private static volatile float visionHz;

    /** \brief Called from the vision path once per processed frame. Cheap and lock-free. */
    public static void reportVisionHz(float hz) {
        if (!(hz > 1f) || hz > 200f) {
            return;
        }
        visionHz = hz;
    }

    public TrafficVisionPipeline(Context context, LaneOverlayView overlay) throws Exception {
        this.overlay = overlay;
        runner = new TrafficYoloRunner(context.getApplicationContext(), "auto");
        thread = new HandlerThread("TrafficYoloInfer", Process.THREAD_PRIORITY_BACKGROUND);
        thread.start();
        handler = new Handler(thread.getLooper());
        Log.i(TAG, "ready lights@" + (1000 / PERIOD_MS) + "Hz (one background thread, one net in flight)");
    }

    /** \brief True when the next camera frame should be copied and submitted. */
    public boolean wantsFrame() {
        synchronized (lock) {
            return !busy && adas.app.TimeUtil.nowMs() - lastStartMs >= PERIOD_MS * backoff;
        }
    }

    /** \brief Takes ownership of {@code frame}; call only after {@link #wantsFrame()}. */
    public void submitYuv(YuvFrame frame, long captureTsMs) {
        if (frame == null) {
            return;
        }
        synchronized (lock) {
            pending = frame;
            pendingTs = captureTsMs > 0 ? captureTsMs : adas.app.TimeUtil.nowMs();
            final long now = adas.app.TimeUtil.nowMs();
            if (busy || now - lastStartMs < PERIOD_MS * backoff) {
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
            updateBackoff();
            TrafficYoloRunner.Result res = runner.run(frame);
            publish(res, id, ts);
            Log.i(TAG, String.format(java.util.Locale.US,
                    "lights total=%dms prep=%d ort=%d decode=%d dets=%d ep=%s vision=%.1fHz x%d",
                    res.inferMs, res.prepMs, res.ortMs, res.decodeMs,
                    res.dets != null ? res.dets.size() : 0, res.ep, visionHz, backoff));
        } catch (Exception e) {
            Log.e(TAG, "infer failed", e);
        } finally {
            synchronized (lock) {
                busy = false;
            }
        }
    }

    /** Widen or restore the period; runs on the worker, so the counters need no lock. */
    private void updateBackoff() {
        final float hz = visionHz;
        final boolean ok = hz <= 0f || hz >= VISION_HEALTH_FRAC * adas.app.sensors.CameraHandler.getTargetFps();
        if (ok) {
            unhealthy = 0;
            if (backoff > 1 && ++healthy >= HEALTHY_TO_RESTORE) {
                healthy = 0;
                backoff = Math.max(1, backoff / 2);
                Log.i(TAG, "vision recovered — detector back to x" + backoff);
            }
            return;
        }
        healthy = 0;
        if (++unhealthy >= UNHEALTHY_TO_BACKOFF && backoff < MAX_BACKOFF) {
            unhealthy = 0;
            backoff = Math.min(MAX_BACKOFF, backoff * 2);
            Log.w(TAG, "vision at " + String.format(java.util.Locale.US, "%.1f", hz)
                    + " Hz — detector period x" + backoff);
        }
    }

    private void publish(TrafficYoloRunner.Result res, int id, long captureTsMs) {
        if (res == null) {
            return;
        }
        if (overlay != null) {
            overlay.setTrafficDets(res.dets);
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
