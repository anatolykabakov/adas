package adas.app.vision;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.View;

import adas.app.vision.overlay.AlertTones;
import adas.app.vision.overlay.CalibrationPainter;
import adas.app.vision.overlay.GroundProjector;
import adas.app.vision.overlay.LeadPainter;
import adas.app.vision.overlay.StatusFramePainter;
import adas.app.vision.overlay.TrafficHudPainter;

/**
 * The windshield view: holds the state the messages deliver, owns the world→screen projection, and decides what is shown.
 */
public class LaneOverlayView extends View {
    private static final float MIN_LANE_PROB = 0.3f;

    /** Every size is written for a 1280-wide view and scaled by {@link #ui}. */
    private static final float REF_WIDTH = 1280f;

    /**
     * Status colours, taken from flowpilot's `statusColors` so a driver moving between the two apps
     * reads the same thing.
     */
    private static final int STATUS_DISENGAGED_RGB = Color.rgb(23, 51, 73);
    private static final int STATUS_ENGAGED_RGB = Color.rgb(43, 143, 40);
    private static final int STATUS_CRITICAL_RGB = Color.rgb(222, 15, 15);
    /** Nothing has arrived from the native side for a while — neither engaged nor safe. */
    private static final int STATUS_STALE_RGB = Color.rgb(120, 120, 120);

    /** Beyond this with no `control/lane_keep`, the lateral loop is not running. */
    private static final long CONTROL_MAX_AGE_MS = 700;

    private final Paint lanePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint edgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint centerCasingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint speedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint speedUnitPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path path = new Path();
    private final Matrix drawMatrix = new Matrix();
    private final float[] mapPt = new float[2];

    private final StatusFramePainter statusFrame = new StatusFramePainter();
    private final LeadPainter leadPainter = new LeadPainter();
    private final TrafficHudPainter trafficHud = new TrafficHudPainter();
    private final CalibrationPainter calibrationPainter = new CalibrationPainter();
    private final AlertTones tones = new AlertTones();

    /** The painters draw on the road through this; the projection itself never leaves the view. */
    private final GroundProjector projector = (x, y, z, xMin, out) -> {
        if (!projectDevice(x, y, z, xMin)) {
            return false;
        }
        out[0] = mapPt[0];
        out[1] = mapPt[1];
        return true;
    };

    private volatile LaneLines lanes;
    private volatile ModelLongParse.Out modelLong;

    private float fx = 930f;
    private float fy = 930f;
    private float cx = 640f;
    private float cy = 360f;
    private float cameraHeight = 1.22f;
    private float frameW = 1280f;
    private float frameH = 720f;
    private float rollDeg = 0f;
    private float pitchDeg = 0f;
    private float yawDeg = 0f;

    /**
     * Camera-frame Rt = V · R(rpy) · V⁻¹ with the same R as {@link ModelCalibWarp}
     * (not LibGDX setFromEulerAnglesRad — that swaps pitch/yaw vs the warp).
     */
    private float r00 = 1, r01 = 0, r02 = 0;
    private float r10 = 0, r11 = 1, r12 = 0;
    private float r20 = 0, r21 = 0, r22 = 1;

    /**
     * The reference line the lateral loop actually drives on: `vision/path`, built in C++ by the Planner out of the model plan and the lane lines, and forwarded here over ZMQ.
     */
    private volatile float[] centerX;
    private volatile float[] centerY;
    private volatile boolean centerAnchored;
    /** Time base is elapsedRealtime, not the message clock: this only answers "is C++ still sending". */
    private volatile long centerAtMs;
    /** Two vision periods. A line older than that is about a road position the car has left. */
    private static final long CENTER_MAX_AGE_MS = 400;
    /** Green while the lane lines hold the line down; amber when only the model plan carries it. */
    private static final int CENTER_ANCHORED_RGB = Color.rgb(0, 230, 118);
    private static final int CENTER_PLAN_ONLY_RGB = Color.rgb(255, 176, 32);

    /** View width / 1280. Recomputed on resize; all drawing multiplies by it. */
    private float ui = 1f;

    /** Camera frame interval, for scaling model velocities. 0 = unknown, no correction applied. */
    private volatile float lastFrameDtMs;

    /** Own speed for the readout, as CAN reports it. */
    private volatile float egoSpeedMps;
    private volatile boolean egoSpeedValid;

    /** elapsedRealtime of the last `control/lane_keep`; 0 = never. */
    private volatile long controlAtMs;

    private volatile boolean steerValid = false;
    private volatile int torqueCnm;
    private volatile boolean steerEnabled;

    /** flowpilot-style on-road alert (FCW / AEB / LDW / torque / overspeed). */
    private volatile boolean alertActive = false;
    private static final String TORQUE_ALERT = "STEERING LIMIT";
    /** How long the assist torque must stay at the ceiling before it is worth saying so. */
    private static final long TORQUE_SAT_HOLD_MS = 1000;
    /** When the current run of saturated ticks began; 0 = not saturated. */
    private long torqueSatSinceMs;
    private volatile String alertText1 = "";
    private volatile String alertText2 = "";
    private volatile int alertBorderRgb = 0; // packed 0xRRGGBB

    /** Set from the params switch; nothing traffic-related is drawn while it is false. */
    private volatile boolean trafficHudEnabled;

    private volatile float[] calibCorners;
    private volatile boolean calibAccepted;
    private volatile boolean calibActive;
    private volatile int calibKept;
    private volatile int calibTarget = 30;
    private volatile String calibMessage = "";

    public LaneOverlayView(Context context) {
        super(context);
        init();
    }

    public LaneOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        lanePaint.setStyle(Paint.Style.STROKE);
        lanePaint.setStrokeWidth(6f);
        lanePaint.setColor(Color.YELLOW);
        edgePaint.setStyle(Paint.Style.STROKE);
        edgePaint.setStrokeWidth(4f);
        edgePaint.setColor(Color.RED);
        // The casing goes under the line so it stays readable on bright asphalt, where a thin green
        // stroke disappears.
        centerCasingPaint.setStyle(Paint.Style.STROKE);
        centerCasingPaint.setStrokeWidth(14f);
        centerCasingPaint.setStrokeCap(Paint.Cap.ROUND);
        centerCasingPaint.setStrokeJoin(Paint.Join.ROUND);
        centerCasingPaint.setColor(Color.argb(150, 0, 0, 0));
        centerPaint.setStyle(Paint.Style.STROKE);
        centerPaint.setStrokeWidth(8f);
        centerPaint.setStrokeCap(Paint.Cap.ROUND);
        centerPaint.setStrokeJoin(Paint.Join.ROUND);
        centerPaint.setColor(CENTER_ANCHORED_RGB);

        textPaint.setColor(Color.rgb(255, 200, 0));
        textPaint.setTextSize(28f);
        textPaint.setTypeface(Typeface.MONOSPACE);
        textBgPaint.setColor(Color.argb(120, 0, 0, 0));
        textBgPaint.setStyle(Paint.Style.FILL);

        // Speed readout, flowpilot's placement and colour: top centre, pale green, unit underneath.
        speedPaint.setColor(Color.rgb(128, 255, 128));
        speedPaint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        speedPaint.setTextAlign(Paint.Align.CENTER);
        speedPaint.setShadowLayer(8f, 0f, 2f, Color.argb(200, 0, 0, 0));
        speedUnitPaint.setColor(Color.rgb(128, 255, 128));
        speedUnitPaint.setTextAlign(Paint.Align.CENTER);
        speedUnitPaint.setShadowLayer(6f, 0f, 2f, Color.argb(200, 0, 0, 0));

        setWillNotDraw(false);
    }

    /**
     * Corners found by the lens calibration, for the driver to see what the detector sees.
     * \param pts Flat x,y pairs in frame pixels; null when the board was not found this frame.
     * \param accepted True when the view was kept — green — rather than rejected as a near-duplicate.
     */
    public void setCalibrationPoints(float[] pts, boolean accepted) {
        this.calibCorners = pts;
        this.calibAccepted = accepted;
        postInvalidate();
    }

    /** Turns the calibration view on and off; off restores the normal lane overlay. */
    public void setCalibrationActive(boolean active) {
        this.calibActive = active;
        if (!active) {
            this.calibCorners = null;
            this.calibMessage = "";
            this.calibKept = 0;
        }
        postInvalidate();
    }

    /**
     * How far the collection has got, and what to tell the driver.
     * \param kept Views collected so far.
     * \param target Views needed.
     * \param message One line of instruction or result; may be empty.
     */
    public void setCalibrationProgress(int kept, int target, String message) {
        this.calibKept = kept;
        this.calibTarget = Math.max(1, target);
        this.calibMessage = message != null ? message : "";
        postInvalidate();
    }

    public void setIntrinsics(float fx, float fy, float cx, float cy, float frameW, float frameH) {
        this.fx = fx;
        if (fy <= 1f || Math.abs(fx / fy - 1f) > 0.15f) {
            this.fy = fx;
        } else {
            this.fy = fy;
        }
        this.cx = cx;
        this.cy = cy;
        this.frameW = frameW;
        this.frameH = frameH;
        updateDrawMatrix();
        postInvalidateOnAnimation();
    }

    /** Calib RPY (deg) — same values / convention as model warp ({@link ModelCalibWarp}). */
    public void setCalibRpyDeg(float rollDeg, float pitchDeg, float yawDeg) {
        this.rollDeg = rollDeg;
        this.pitchDeg = pitchDeg;
        this.yawDeg = yawDeg;
        rebuildRt();
        postInvalidateOnAnimation();
    }

    public void setPreviewTransform(Matrix transform, int bufferW, int bufferH, int viewWidth, int viewHeight) {
        this.frameW = bufferW;
        this.frameH = bufferH;
        updateDrawMatrix();
        postInvalidateOnAnimation();
    }

    public void setCameraHeight(float meters) {
        this.cameraHeight = meters;
        postInvalidateOnAnimation();
    }

    public void setLanes(LaneLines lanes) {
        this.lanes = lanes == null ? null : lanes.copy();
        postInvalidateOnAnimation();
    }

    public void setModelLong(ModelLongParse.Out modelLong) {
        this.modelLong = modelLong;
        postInvalidateOnAnimation();
    }

    /** Camera period, needed to read the model's velocities. */
    public void setFrameDtMs(float dtMs) {
        this.lastFrameDtMs = dtMs;
    }

    /**
     * A `control/lane_keep` arrived. Only its arrival time is kept — it feeds the stale colour of the
     * status frame. The geometry it carries used to drive a pure-pursuit debug HUD that was removed
     * from the screen; it lives in the bag.
     */
    public void setLaneKeep(boolean hasTarget, float targetX, float targetY,
                            float lookaheadM, float curvature, float steerRad, String status) {
        this.controlAtMs = android.os.SystemClock.elapsedRealtime();
        postInvalidateOnAnimation();
    }

    /** The reference line from `vision/path` (C++ Planner), in the device frame: x forward, y right. */
    public void setCenterline(float[] xs, float[] ys, boolean anchored) {
        if (xs == null || ys == null || xs.length < 2 || ys.length < 2) {
            clearCenterline();
            return;
        }
        this.centerX = xs;
        this.centerY = ys;
        this.centerAnchored = anchored;
        this.centerAtMs = android.os.SystemClock.elapsedRealtime();
        postInvalidateOnAnimation();
    }

    public void clearCenterline() {
        this.centerX = null;
        this.centerY = null;
        this.centerAtMs = 0;
        postInvalidateOnAnimation();
    }

    /** Own speed from `vehicle/state`, m/s. Displayed in km/h, top centre. */
    public void setEgoSpeed(float mps) {
        this.egoSpeedMps = mps;
        this.egoSpeedValid = true;
        postInvalidateOnAnimation();
    }

    public void setSteerCommand(int torqueCnm, boolean enabled) {
        this.torqueCnm = torqueCnm;
        this.steerEnabled = enabled;
        this.steerValid = true;
        postInvalidateOnAnimation();
    }

    /**
     * flowpilot OnRoadScreen-style alert: colored border + two-line label.
     * Priority: AEB &gt; FCW &gt; LDW. Cleared when all flags false.
     */
    public void setSafetyWarn(boolean fcw, boolean aeb, boolean lldw, boolean rldw) {
        final String was = alertText1;
        final boolean wasActive = alertActive;
        if (aeb) {
            alertActive = true;
            alertText1 = "BRAKE!";
            alertText2 = "Risk of Collision";
            alertBorderRgb = 0xDE0F0F; // STATUS_ALERT
        } else if (fcw) {
            alertActive = true;
            alertText1 = "BRAKE!";
            alertText2 = "Risk of Collision";
            alertBorderRgb = 0xDA6F25; // STATUS_WARNING
        } else if (lldw || rldw) {
            alertActive = true;
            alertText1 = "Lane Departure Detected";
            alertText2 = lldw && rldw ? "Left & Right" : (lldw ? "Left" : "Right");
            alertBorderRgb = 0xDA6F25;
        } else {
            alertActive = false;
            alertText1 = "";
            alertText2 = "";
        }
        if (alertActive != wasActive || !alertText1.equals(was)) {
            updateAlertSound(aeb || fcw);
        }
        postInvalidateOnAnimation();
    }

    /** Assist torque at the MQB ceiling: the controller wants more than the power steering gives,
     *  so the car is about to run wide. Lower priority than a collision or a departure warning —
     *  it only speaks when nothing more urgent is on screen, and only after sustained saturation
     *  so a single clipped frame does not flash the border. */
    public void setTorqueSaturated(boolean saturated) {
        final long now = android.os.SystemClock.elapsedRealtime();
        if (!saturated) {
            torqueSatSinceMs = 0;
        } else if (torqueSatSinceMs == 0) {
            torqueSatSinceMs = now;
        }
        final boolean show = torqueSatSinceMs != 0 && now - torqueSatSinceMs >= TORQUE_SAT_HOLD_MS;
        if (show && (!alertActive || TORQUE_ALERT.equals(alertText1))) {
            final boolean isNew = !TORQUE_ALERT.equals(alertText1);
            alertActive = true;
            alertText1 = TORQUE_ALERT;
            alertText2 = "Assist at max torque";
            alertBorderRgb = 0xDA6F25; // STATUS_WARNING
            if (isNew) {
                updateAlertSound(false);
            }
            postInvalidateOnAnimation();
        } else if (!show && TORQUE_ALERT.equals(alertText1)) {
            alertActive = false;
            alertText1 = "";
            alertText2 = "";
            stopSounds();
            postInvalidateOnAnimation();
        }
    }

    /** Whether detected signs and lights are drawn. */
    public void setTrafficHudEnabled(boolean on) {
        this.trafficHudEnabled = on;
        if (!on) {
            trafficHud.setDets(null);
        }
        postInvalidateOnAnimation();
    }

    public void setTrafficDets(java.util.List<TrafficYoloRunner.Det> dets) {
        trafficHud.setDets(dets);
        postInvalidateOnAnimation();
    }

    public void setTrafficVision(int tflColor, float tflConf) {
        trafficHud.setVision(tflColor, tflConf);
        postInvalidateOnAnimation();
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        applyUiScale(w);
        updateDrawMatrix();
    }

    /** Text and stroke sizes for this screen. Called on resize, so nothing is fixed in raw pixels. */
    private void applyUiScale(int width) {
        ui = width > 0 ? width / REF_WIDTH : 1f;
        lanePaint.setStrokeWidth(6f * ui);
        edgePaint.setStrokeWidth(4f * ui);
        centerPaint.setStrokeWidth(8f * ui);
        centerCasingPaint.setStrokeWidth(14f * ui);
        speedPaint.setTextSize(100f * ui);
        speedUnitPaint.setTextSize(26f * ui);
        statusFrame.onUiScale(ui);
    }

    private void updateDrawMatrix() {
        if (getWidth() <= 0 || getHeight() <= 0 || frameW <= 0 || frameH <= 0) {
            return;
        }
        float scale = Math.max(getWidth() / frameW, getHeight() / frameH);
        float dx = (getWidth() - frameW * scale) * 0.5f;
        float dy = (getHeight() - frameH * scale) * 0.5f;
        drawMatrix.reset();
        drawMatrix.setScale(scale, scale);
        drawMatrix.postTranslate(dx, dy);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        // Lens calibration takes over the view entirely: the lane overlay is drawn from intrinsics we
        // are in the middle of measuring, so showing it here would be showing the answer to the
        // question being asked.
        if (calibCorners != null || calibActive) {
            calibrationPainter.draw(canvas, getWidth(), getHeight(), frameW, frameH,
                    calibCorners, calibAccepted, calibKept, calibTarget, calibMessage);
            return;
        }
        LaneLines ll = this.lanes;
        if (ll != null) {
            for (int i = 0; i < 2; i++) {
                drawPolylineXYZ(canvas, LaneLines.X_IDXS, ll.edgesY[i], ll.edgesZ[i],
                        edgePaint, 1.5f);
            }
            for (int i = 0; i < 4; i++) {
                if (ll.laneProbs[i] < MIN_LANE_PROB) {
                    continue;
                }
                lanePaint.setAlpha((int) (80 + 175 * Math.max(0f, Math.min(1f, ll.laneProbs[i]))));
                drawPolylineXYZ(canvas, LaneLines.X_IDXS, ll.lanesY[i], ll.lanesZ[i],
                        lanePaint, 1.5f);
            }
            leadPainter.draw(canvas, modelLong, lastFrameDtMs, egoSpeedValid, egoSpeedMps,
                    cameraHeight, ui, getWidth(), getHeight(), projector);
            drawLatencyHud(canvas, ll);
        }

        drawCenterline(canvas);
        // The status frame is drawn on every frame, alert or not: it is the one thing on screen that
        // answers "is this system running and is it steering", and a colour is only noticed as a
        // change if it was already there.
        statusFrame.draw(canvas, getWidth(), getHeight(), ui, statusRgb(),
                alertActive ? alertText1 : "", alertActive ? alertText2 : "");
        if (egoSpeedValid) {
            drawSpeed(canvas);
        }
        // Signs and lights come back on screen only with their switch; the timing block next to them
        // stays off — it belongs in the bag, not on the windshield.
        if (trafficHudEnabled) {
            trafficHud.draw(canvas, getWidth(), getHeight(), ui);
        }
        // Removed on request: the model plan and the pure-pursuit geometry (lookahead circle, curvature
        // arc, target ray), then the PP debug lines and panda status, traffic-light overlays and
        // sign-detector timings. All of it goes into the bag and is parsed offline, while on screen it
        // covered the road and showed an intermediate rather than the line the car is driving.
    }

    /**
     * What the frame colour means. An alert outranks the engagement state, and a silent native side
     * outranks both — "nothing on screen" used to be indistinguishable from "everything is fine".
     */
    private int statusRgb() {
        if (alertActive) {
            return alertBorderRgb == 0 ? STATUS_CRITICAL_RGB
                    : Color.rgb((alertBorderRgb >> 16) & 0xFF, (alertBorderRgb >> 8) & 0xFF,
                                alertBorderRgb & 0xFF);
        }
        final long age = android.os.SystemClock.elapsedRealtime() - controlAtMs;
        if (controlAtMs == 0 || age > CONTROL_MAX_AGE_MS) {
            return STATUS_STALE_RGB;
        }
        return steerValid && steerEnabled ? STATUS_ENGAGED_RGB : STATUS_DISENGAGED_RGB;
    }

    /** Own speed, big, top centre — the number a driver checks without looking away for long. */
    private void drawSpeed(Canvas canvas) {
        final float kmh = egoSpeedMps * 3.6f;
        final float cx = getWidth() * 0.5f;
        final float y = 96f * ui;
        canvas.drawText(String.valueOf(Math.round(kmh)), cx, y, speedPaint);
        canvas.drawText("km/h", cx, y + 30f * ui, speedUnitPaint);
    }

    /** Capture → model output, the only latency worth a place on the windshield. */
    private void drawLatencyHud(Canvas canvas, LaneLines ll) {
        if (ll.captureTimestampMs <= 0 || ll.inferTimestampMs <= ll.captureTimestampMs) {
            return;
        }
        final float e2eMs = (float) (ll.inferTimestampMs - ll.captureTimestampMs);
        final String line = String.format(java.util.Locale.US, "e2e %4.0f ms", e2eMs);

        final float pad = 10f * ui;
        textPaint.setTextSize(24f * ui);
        textPaint.setTypeface(Typeface.MONOSPACE);
        final float tw = textPaint.measureText(line);
        final float x = getWidth() - tw - pad - 12f * ui;
        final float y = 40f * ui;
        canvas.drawRoundRect(x - pad, y - 28f * ui, x + tw + pad, y + 8f * ui, 8f * ui, 8f * ui,
                textBgPaint);
        textPaint.setColor(Color.rgb(255, 200, 80));
        canvas.drawText(line, x, y, textPaint);
    }

    /** The line the car is driving on, drawn on the road plane. */
    private void drawCenterline(Canvas canvas) {
        final float[] xs = this.centerX;
        final float[] ys = this.centerY;
        if (xs == null || ys == null) {
            return;
        }
        // A line that stopped arriving must stop being drawn: a frozen path on screen is
        // indistinguishable from a live one, and reads as "the system still knows where to go".
        if (android.os.SystemClock.elapsedRealtime() - centerAtMs > CENTER_MAX_AGE_MS) {
            return;
        }
        path.reset();
        boolean started = false;
        final int n = Math.min(xs.length, ys.length);
        for (int i = 0; i < n; i++) {
            if (!projectDevice(xs[i], ys[i], cameraHeight, 1.0f)) {
                started = false;
                continue;
            }
            if (!started) {
                path.moveTo(mapPt[0], mapPt[1]);
                started = true;
            } else {
                path.lineTo(mapPt[0], mapPt[1]);
            }
        }
        if (!started) {
            return;
        }
        centerPaint.setColor(centerAnchored ? CENTER_ANCHORED_RGB : CENTER_PLAN_ONLY_RGB);
        canvas.drawPath(path, centerCasingPaint);
        canvas.drawPath(path, centerPaint);
    }

    private void drawPolylineXYZ(Canvas canvas, float[] xs, float[] ys, float[] zs,
                                 Paint paint, float xMin) {
        path.reset();
        boolean started = false;
        int n = Math.min(xs.length, ys.length);
        if (zs != null) {
            n = Math.min(n, zs.length);
        }
        for (int i = 0; i < n; i++) {
            float X = xs[i];
            float Z = zs != null ? zs[i] : 0f;
            if (X < xMin || !Float.isFinite(X) || !Float.isFinite(ys[i]) || !Float.isFinite(Z)) {
                started = false;
                continue;
            }
            if (!projectDevice(X, ys[i], Z, xMin)) {
                started = false;
                continue;
            }
            float px = mapPt[0];
            float py = mapPt[1];
            if (!started) {
                path.moveTo(px, py);
                started = true;
            } else {
                path.lineTo(px, py);
            }
        }
        if (started) {
            canvas.drawPath(path, paint);
        }
    }

    /**
     * Remap device (X,Y,Z)→(Y,Z,X) then apply Rt. Device: X fwd, Y right+, Z down.
     * Rt matches warp: V·R(rpy)·V⁻¹ with R from {@link ModelCalibWarp#rotFromEuler}.
     */
    private boolean projectDevice(float X, float Y, float Z, float xMin) {
        if (X < xMin || !Float.isFinite(X) || !Float.isFinite(Y) || !Float.isFinite(Z)) {
            return false;
        }
        float camX = Y;
        float camY = Z;
        float camZ = X;
        float x = r00 * camX + r01 * camY + r02 * camZ;
        float y = r10 * camX + r11 * camY + r12 * camZ;
        float z = r20 * camX + r21 * camY + r22 * camZ;
        if (z < 0.15f) {
            return false;
        }
        float u = fx * (x / z) + cx;
        float v = fy * (y / z) + cy;
        mapPt[0] = u;
        mapPt[1] = v;
        drawMatrix.mapPoints(mapPt);
        float px = mapPt[0];
        float py = mapPt[1];
        return !(px < -80 || px > getWidth() + 80 || py < -80 || py > getHeight() + 80);
    }

    @Override
    protected void onDetachedFromWindow() {
        // The view can go away mid-alert (screen off, activity destroyed). A looping tone that
        // outlives the view is worse than no tone at all.
        tones.release();
        super.onDetachedFromWindow();
    }

    /** Silence everything — for leaving the road, stopping the pipeline, or losing the view. */
    public void stopSounds() {
        tones.stop();
    }

    /** Start, change or stop the alert sound to match what is on screen. */
    private void updateAlertSound(boolean critical) {
        if (!alertActive) {
            tones.stop();
            return;
        }
        tones.play(critical);
    }

    /**
     * Keep overlay RPY axes identical to {@link ModelCalibWarp} / Preprocess.
     * LibGDX {@code setFromEulerAnglesRad(-pitch,-yaw,-roll)} mixes pitch↔yaw
     * relative to that convention (slider Pitch looked like Yaw on the HUD).
     */
    private void rebuildRt() {
        float[] R = ModelCalibWarp.rotFromEuler(
                Math.toRadians(rollDeg),
                Math.toRadians(pitchDeg),
                Math.toRadians(yawDeg));
        // view_from_device
        float[] V = {
                0, 1, 0,
                0, 0, 1,
                1, 0, 0
        };
        // V⁻¹ = Vᵀ for this permutation matrix
        float[] Vi = {
                0, 0, 1,
                1, 0, 0,
                0, 1, 0
        };
        float[] Rt = ModelCalibWarp.mul3(ModelCalibWarp.mul3(V, R), Vi);
        r00 = Rt[0];
        r01 = Rt[1];
        r02 = Rt[2];
        r10 = Rt[3];
        r11 = Rt[4];
        r12 = Rt[5];
        r20 = Rt[6];
        r21 = Rt[7];
        r22 = Rt[8];
    }
}
