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

public class LaneOverlayView extends View {
    private static final float MIN_LANE_PROB = 0.3f;
    private static final float MAX_TORQUE_CNM = 300f;

    private final Paint lanePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint edgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ppArcPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ppLdPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ppTargetPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ppRayPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint hudPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint hudFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint leadPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint leadTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path path = new Path();
    private final Matrix drawMatrix = new Matrix();
    private final float[] mapPt = new float[2];

    private volatile LaneLines lanes;
    private volatile ModelLongParse.Out modelLong;

    private float fx = 930f;
    private float fy = 930f;
    private float cx = 640f;
    private float cy = 360f;
    private float cameraHeight = 1.22f;
    private float frameW = 1280f;
    private float frameH = 720f;
    private float waypointShift = 1.40f;
    private float steerRatio = 15.7f;
    private float rollDeg = 0f;
    private float pitchDeg = 0f;
    private float yawDeg = 0f;

    /** flowpilot OnRoadScreen path lift (m) on remapped camera-up axis. */
    private static final float PATH_LIFT_M = 1.28f;

    /**
     * Camera-frame Rt = V · R(rpy) · V⁻¹ with the same R as {@link ModelCalibWarp}
     * (not LibGDX setFromEulerAnglesRad — that swaps pitch/yaw vs the warp).
     */
    private float r00 = 1, r01 = 0, r02 = 0;
    private float r10 = 0, r11 = 1, r12 = 0;
    private float r20 = 0, r21 = 0, r22 = 1;


    private volatile boolean ppValid = false;
    private volatile boolean ppHasTarget = false;
    private volatile float ppTargetX;
    private volatile float ppTargetY;
    private volatile float ppLookaheadM;
    private volatile float ppCurvature;
    private volatile float ppSteerRad;
    private volatile String ppStatus = "";


    private volatile boolean steerValid = false;
    private volatile String hcaStatus = "";
    private volatile boolean hcaValid = false;
    private volatile int torqueCnm;
    private volatile boolean steerEnabled;

    /** flowpilot-style on-road alert (FCW / AEB / LDW). */
    private volatile boolean alertActive = false;
    private static final String TORQUE_ALERT = "STEERING LIMIT";
    /** control/lane_keep arrives once per vision frame (~11 Hz), so 5 frames ≈ 0.45 s. */
    private static final int TORQUE_SAT_FRAMES = 5;
    private int torqueSatFrames = 0;
    private volatile String alertText1 = "";
    private volatile String alertText2 = "";
    private volatile int alertBorderRgb = 0; // packed 0xRRGGBB
    private final Paint alertBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint alertText1Paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint alertText2Paint = new Paint(Paint.ANTI_ALIAS_FLAG);

    /** Traffic YOLO dets + fused HUD (limit / TFL / overspeed). */
    private volatile java.util.List<TrafficYoloRunner.Det> trafficDets =
            java.util.Collections.emptyList();
    private volatile int speedLimitKmh = 0;
    private volatile float vEgoKmh = 0f;
    private volatile boolean overspeed = false;
    private volatile float overspeedKmh = 0f;
    private volatile int tflColor = 0; // TrafficLightColor number
    private volatile float tflConf = 0f;
    private volatile int trafficPrepMs;
    private volatile int trafficOrtMs;
    private volatile int trafficDecodeMs;
    private volatile int trafficOcrMs;
    private volatile int trafficTotalMs;
    private volatile int trafficE2eMs;
    private final Paint tflPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint limitPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint detBoxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

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
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(8f);
        pathPaint.setColor(Color.GREEN);

        ppArcPaint.setStyle(Paint.Style.STROKE);
        ppArcPaint.setStrokeWidth(7f);
        ppArcPaint.setColor(Color.MAGENTA);
        ppLdPaint.setStyle(Paint.Style.STROKE);
        ppLdPaint.setStrokeWidth(2.5f);
        ppLdPaint.setColor(Color.rgb(255, 128, 0));
        ppTargetPaint.setStyle(Paint.Style.FILL);
        ppTargetPaint.setColor(Color.CYAN);
        ppRayPaint.setStyle(Paint.Style.STROKE);
        ppRayPaint.setStrokeWidth(3f);
        ppRayPaint.setColor(Color.CYAN);

        hudPaint.setStyle(Paint.Style.STROKE);
        hudPaint.setStrokeWidth(3f);
        hudPaint.setColor(Color.rgb(220, 220, 220));
        hudFillPaint.setStyle(Paint.Style.FILL);
        hudFillPaint.setColor(Color.argb(140, 30, 30, 30));

        textPaint.setColor(Color.rgb(255, 200, 0));
        textPaint.setTextSize(28f);
        textPaint.setTypeface(Typeface.MONOSPACE);

        alertBorderPaint.setStyle(Paint.Style.STROKE);
        alertBorderPaint.setStrokeWidth(28f);
        alertText1Paint.setColor(Color.WHITE);
        alertText1Paint.setTextSize(52f);
        alertText1Paint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        alertText1Paint.setTextAlign(Paint.Align.CENTER);
        alertText1Paint.setShadowLayer(6f, 0f, 2f, Color.argb(180, 0, 0, 0));
        alertText2Paint.setColor(Color.WHITE);
        alertText2Paint.setTextSize(28f);
        alertText2Paint.setTypeface(Typeface.DEFAULT);
        alertText2Paint.setTextAlign(Paint.Align.CENTER);
        alertText2Paint.setShadowLayer(4f, 0f, 1f, Color.argb(160, 0, 0, 0));

        detBoxPaint.setStyle(Paint.Style.STROKE);
        detBoxPaint.setStrokeWidth(3f);
        detBoxPaint.setColor(Color.rgb(255, 200, 40));
        tflPaint.setStyle(Paint.Style.FILL);
        limitPaint.setStyle(Paint.Style.FILL);
        limitPaint.setColor(Color.WHITE);
        limitPaint.setTextAlign(Paint.Align.CENTER);
        limitPaint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        limitPaint.setTextSize(36f);
        textBgPaint.setColor(Color.argb(120, 0, 0, 0));
        textBgPaint.setStyle(Paint.Style.FILL);

        leadPaint.setStyle(Paint.Style.STROKE);
        leadPaint.setStrokeWidth(5f);
        leadPaint.setColor(Color.rgb(0, 200, 255));
        leadTextPaint.setColor(Color.rgb(0, 220, 255));
        leadTextPaint.setTextSize(26f);
        leadTextPaint.setTypeface(Typeface.MONOSPACE);

        setWillNotDraw(false);
    }

    private volatile float[] calibCorners;
    private volatile boolean calibAccepted;
    private volatile boolean calibActive;
    private volatile int calibKept;
    private volatile int calibTarget = 30;
    private volatile String calibMessage = "";
    private final Paint calibPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint calibTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint calibPanelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    /**
     * Corners found by the lens calibration, for the driver to see what the detector sees.
     *
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
     *
     * <p>On screen rather than in a toast: a toast queues, lags behind the camera and is gone before it
     * is read, which is exactly wrong for something the driver is meant to react to while holding a
     * board in front of the lens.
     *
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

    private void drawCalibrationPanel(Canvas canvas) {
        final float pad = 24f;
        final float barH = 18f;
        final float panelH = 132f;
        final float w = getWidth();

        calibPanelPaint.setColor(Color.argb(190, 0, 0, 0));
        canvas.drawRect(0f, 0f, w, panelH, calibPanelPaint);

        calibTextPaint.setColor(Color.WHITE);
        calibTextPaint.setTextSize(40f);
        calibTextPaint.setFakeBoldText(true);
        canvas.drawText(calibKept + " / " + calibTarget, pad, 50f, calibTextPaint);

        calibTextPaint.setTextSize(26f);
        calibTextPaint.setFakeBoldText(false);
        final String msg = calibMessage;
        if (!msg.isEmpty()) {
            canvas.drawText(msg, pad + 160f, 46f, calibTextPaint);
        }

        // Progress bar: how many views have been accepted out of the number required.
        final float barY = panelH - pad - barH;
        calibPanelPaint.setColor(Color.argb(120, 255, 255, 255));
        canvas.drawRect(pad, barY, w - pad, barY + barH, calibPanelPaint);
        calibPanelPaint.setColor(calibKept >= calibTarget ? Color.GREEN : Color.rgb(0, 170, 255));
        final float frac = Math.min(1f, calibKept / (float) calibTarget);
        canvas.drawRect(pad, barY, pad + (w - 2 * pad) * frac, barY + barH, calibPanelPaint);
    }

    private void drawCalibration(Canvas canvas) {
        drawCalibrationPanel(canvas);
        final float[] pts = calibCorners;
        if (pts == null || pts.length < 2) {
            return;
        }
        final float sx = getWidth() / frameW;
        final float sy = getHeight() / frameH;
        calibPaint.setColor(calibAccepted ? Color.GREEN : Color.rgb(255, 176, 0));
        calibPaint.setStyle(Paint.Style.STROKE);
        calibPaint.setStrokeWidth(3f);
        for (int i = 0; i + 3 < pts.length; i += 2) {
            canvas.drawLine(pts[i] * sx, pts[i + 1] * sy, pts[i + 2] * sx, pts[i + 3] * sy, calibPaint);
        }
        calibPaint.setStyle(Paint.Style.FILL);
        for (int i = 0; i + 1 < pts.length; i += 2) {
            canvas.drawCircle(pts[i] * sx, pts[i + 1] * sy, 7f, calibPaint);
        }
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


    public void setWaypointShift(float meters) {
        this.waypointShift = meters;
        postInvalidateOnAnimation();
    }

    public void setSteerRatio(float ratio) {
        this.steerRatio = ratio > 1f ? ratio : 15.7f;
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


    public void setLaneKeep(boolean hasTarget, float targetX, float targetY,
                            float lookaheadM, float curvature, float steerRad, String status) {
        this.ppHasTarget = hasTarget;
        this.ppTargetX = targetX;
        this.ppTargetY = targetY;
        this.ppLookaheadM = lookaheadM;
        this.ppCurvature = curvature;
        this.ppSteerRad = steerRad;
        this.ppStatus = status == null ? "" : status;
        this.ppValid = true;
        postInvalidateOnAnimation();
    }


    public void setSteerCommand(int torqueCnm, boolean enabled) {
        this.torqueCnm = torqueCnm;
        this.steerEnabled = enabled;
        this.steerValid = true;
        postInvalidateOnAnimation();
    }

    public void setHcaStatus(String status) {
        this.hcaStatus = status == null ? "" : status;
        this.hcaValid = true;
        postInvalidateOnAnimation();
    }

    /**
     * flowpilot OnRoadScreen-style alert: colored border + two-line label.
     * Priority: AEB &gt; FCW &gt; LDW. Cleared when all flags false.
     */
    public void setSafetyWarn(boolean fcw, boolean aeb, boolean lldw, boolean rldw) {
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
        postInvalidateOnAnimation();
    }

    /** Assist torque at the MQB ceiling: the controller wants more than the power steering gives,
     *  so the car is about to run wide. Lower priority than a collision or a departure warning —
     *  it only speaks when nothing more urgent is on screen, and only after ~0.5 s of saturation
     *  so a single clipped frame does not flash the border. */
    public void setTorqueSaturated(boolean saturated) {
        torqueSatFrames = saturated ? torqueSatFrames + 1 : 0;
        final boolean show = torqueSatFrames >= TORQUE_SAT_FRAMES;
        if (show && (!alertActive || TORQUE_ALERT.equals(alertText1))) {
            alertActive = true;
            alertText1 = TORQUE_ALERT;
            alertText2 = "Assist at max torque";
            alertBorderRgb = 0xDA6F25; // STATUS_WARNING
            postInvalidateOnAnimation();
        } else if (!show && TORQUE_ALERT.equals(alertText1)) {
            alertActive = false;
            alertText1 = "";
            alertText2 = "";
            postInvalidateOnAnimation();
        }
    }

    public void setTrafficDets(java.util.List<TrafficYoloRunner.Det> dets) {
        this.trafficDets = dets != null ? dets : java.util.Collections.emptyList();
        postInvalidateOnAnimation();
    }

    /** YOLO latency for HUD (also logged in vision/traffic_dets bag). */
    public void setTrafficLatency(int prepMs, int ortMs, int decodeMs, int ocrMs, int totalMs, int e2eMs) {
        this.trafficPrepMs = prepMs;
        this.trafficOrtMs = ortMs;
        this.trafficDecodeMs = decodeMs;
        this.trafficOcrMs = ocrMs;
        this.trafficTotalMs = totalMs;
        this.trafficE2eMs = e2eMs;
        postInvalidateOnAnimation();
    }

    public void setTrafficVision(int speedLimitKmh, float vEgoKmh, boolean overspeed, float overspeedKmh,
                                 int tflColor, float tflConf, String status) {
        this.speedLimitKmh = speedLimitKmh;
        this.vEgoKmh = vEgoKmh;
        this.overspeed = overspeed;
        this.overspeedKmh = overspeedKmh;
        this.tflColor = tflColor;
        this.tflConf = tflConf;
        if (overspeed) {
            alertActive = true;
            alertText1 = "SPEED!";
            alertText2 = String.format(java.util.Locale.US, "%.0f > %d km/h", vEgoKmh, speedLimitKmh);
            alertBorderRgb = 0xDE0F0F;
        } else if ("SPEED!".equals(alertText1)) {
            alertActive = false;
            alertText1 = "";
            alertText2 = "";
        }
        postInvalidateOnAnimation();
    }

    public void clearLaneKeep() {
        ppValid = false;
        steerValid = false;
        postInvalidateOnAnimation();
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        updateDrawMatrix();
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
            drawCalibration(canvas);
            return;
        }
        LaneLines ll = this.lanes;
        if (ll != null) {
            for (int i = 0; i < 2; i++) {
                drawPolylineXYZ(canvas, LaneLines.X_IDXS, ll.edgesY[i], ll.edgesZ[i],
                        edgePaint, 1.5f, false);
            }
            for (int i = 0; i < 4; i++) {
                if (ll.laneProbs[i] < MIN_LANE_PROB) {
                    continue;
                }
                lanePaint.setAlpha((int) (80 + 175 * Math.max(0f, Math.min(1f, ll.laneProbs[i]))));
                drawPolylineXYZ(canvas, LaneLines.X_IDXS, ll.lanesY[i], ll.lanesZ[i],
                        lanePaint, 1.5f, false);
            }
            if (ll.hasPlan) {
                drawPolylineXYZ(canvas, ll.planX, ll.planY, ll.planZ, pathPaint, 0.5f, true);
            }
            drawLead(canvas);
            drawLatencyHud(canvas, ll);
        }

        if (ppValid) {
            drawPurePursuit(canvas);
        }
        if (alertActive) {
            drawSafetyAlert(canvas);
        }
        // Removed on request: the PP debug lines and panda status, plus traffic-light overlays and
        // sign-detector timings. All of it goes into the bag and is parsed offline, while on screen it
        // covered the road. The methods stay — bringing them back means bringing back one call.
    }

    private void drawTrafficLatencyHud(Canvas canvas) {
        if (trafficTotalMs <= 0 && trafficOrtMs <= 0) {
            return;
        }
        String line1 = String.format(java.util.Locale.US,
                "yolo p%3d o%3d d%3d ocr%2d",
                trafficPrepMs, trafficOrtMs, trafficDecodeMs, trafficOcrMs);
        String line2 = String.format(java.util.Locale.US,
                "yolo tot%3d e2e%3d", trafficTotalMs, trafficE2eMs);
        float pad = 10f;
        textPaint.setTextSize(22f);
        textPaint.setTypeface(Typeface.MONOSPACE);
        float tw = Math.max(textPaint.measureText(line1), textPaint.measureText(line2));
        float x = getWidth() - tw - pad - 12f;
        // Under supercombo latency block (which starts ~y=40, height ~58)
        float y = 112f;
        float h = 54f;
        canvas.drawRoundRect(x - pad, y - 24f, x + tw + pad, y - 24f + h, 8f, 8f, textBgPaint);
        textPaint.setColor(Color.rgb(180, 255, 160));
        canvas.drawText(line1, x, y, textPaint);
        textPaint.setColor(Color.rgb(255, 180, 100));
        canvas.drawText(line2, x, y + 24f, textPaint);
    }

    private void drawTrafficHud(Canvas canvas) {
        int w = getWidth();
        int h = getHeight();
        if (w <= 0 || h <= 0) {
            return;
        }
        // Detection boxes (normalized → view)
        java.util.List<TrafficYoloRunner.Det> dets = this.trafficDets;
        if (dets != null) {
            for (TrafficYoloRunner.Det d : dets) {
                float l = d.x1 * w;
                float t = d.y1 * h;
                float r = d.x2 * w;
                float b = d.y2 * h;
                String lab = d.label != null ? d.label.toLowerCase(java.util.Locale.US) : "";
                int col;
                if (d.tflColor == 1) {
                    col = Color.RED;
                } else if (d.tflColor == 2) {
                    col = Color.YELLOW;
                } else if (d.tflColor == 3) {
                    col = Color.GREEN;
                } else if (lab.contains("переход")) {
                    col = Color.rgb(255, 220, 60); // crosswalk
                } else if (lab.equals("person") || lab.contains("пешеход") && !lab.contains("переход")
                        || lab.contains("pedestrian")) {
                    col = Color.rgb(80, 200, 255); // people
                } else if (lab.equals("bicycle") || lab.equals("motorcycle") || lab.contains("велосипед")) {
                    col = Color.rgb(180, 120, 255);
                } else if (lab.contains("stop") || lab.contains("sign") || lab.contains("ограничен")
                        || lab.contains("запрещ") || lab.contains("знак") || d.speedLimitKmh > 0
                        || lab.contains("уступи") || lab.contains("главн") || lab.contains("дети")) {
                    col = Color.rgb(255, 90, 90); // signs
                } else {
                    col = Color.rgb(255, 200, 40);
                }
                detBoxPaint.setColor(col);
                canvas.drawRect(l, t, r, b, detBoxPaint);
                textPaint.setTextSize(22f);
                textPaint.setColor(col);
                String tag = d.label;
                if (d.speedLimitKmh > 0) {
                    tag = tag + " " + d.speedLimitKmh + (d.speedFromOcr ? " OCR" : "");
                }
                canvas.drawText(tag + String.format(java.util.Locale.US, " %.2f", d.conf),
                        l + 4f, Math.max(24f, t - 6f), textPaint);
            }
        }

        // Speed-limit disc (top-right)
        if (speedLimitKmh > 0) {
            float cx = w - 72f;
            float cy = 78f;
            float rad = 48f;
            Paint ring = tflPaint;
            ring.setColor(overspeed ? Color.rgb(220, 40, 40) : Color.rgb(220, 220, 220));
            canvas.drawCircle(cx, cy, rad, ring);
            ring.setColor(Color.WHITE);
            canvas.drawCircle(cx, cy, rad - 8f, ring);
            limitPaint.setColor(Color.BLACK);
            limitPaint.setTextSize(40f);
            canvas.drawText(String.valueOf(speedLimitKmh), cx, cy + 14f, limitPaint);
            if (overspeed) {
                textPaint.setTextSize(22f);
                textPaint.setColor(Color.rgb(255, 80, 80));
                canvas.drawText(String.format(java.util.Locale.US, "+%.0f", overspeedKmh),
                        cx - 20f, cy + rad + 22f, textPaint);
            }
        }

        // Current traffic light (top-left under HCA)
        float lx = 36f;
        float ly = ppValid ? 160f : 90f;
        int[] cols = {Color.rgb(60, 60, 60), Color.RED, Color.YELLOW, Color.GREEN, Color.rgb(40, 40, 40)};
        int active = Math.max(0, Math.min(4, tflColor));
        // housing
        tflPaint.setColor(Color.rgb(20, 20, 20));
        canvas.drawRoundRect(lx - 18f, ly - 18f, lx + 18f, ly + 70f, 8f, 8f, tflPaint);
        for (int i = 1; i <= 3; i++) {
            float yy = ly + (i - 1) * 28f;
            boolean on = (active == i);
            tflPaint.setColor(on ? cols[i] : Color.rgb(50, 50, 50));
            canvas.drawCircle(lx, yy, on ? 11f : 9f, tflPaint);
        }
        if (tflConf > 0.01f) {
            textPaint.setTextSize(18f);
            textPaint.setColor(Color.WHITE);
            canvas.drawText(String.format(java.util.Locale.US, "%.0f%%", tflConf * 100f),
                    lx + 24f, ly + 8f, textPaint);
        }
    }

    /** Mirror flowpilot OnRoadScreen.drawAlert: full-bleed tinted border + bottom labels. */
    private void drawSafetyAlert(Canvas canvas) {
        int w = getWidth();
        int h = getHeight();
        if (w <= 0 || h <= 0) {
            return;
        }
        int r = (alertBorderRgb >> 16) & 0xFF;
        int g = (alertBorderRgb >> 8) & 0xFF;
        int b = alertBorderRgb & 0xFF;
        alertBorderPaint.setColor(Color.argb(180, r, g, b));
        float half = alertBorderPaint.getStrokeWidth() * 0.5f;
        canvas.drawRect(half, half, w - half, h - half, alertBorderPaint);

        float cx = w * 0.5f;
        float y1 = h - 88f;
        float y2 = h - 48f;
        if (!alertText1.isEmpty()) {
            canvas.drawText(alertText1, cx, y1, alertText1Paint);
        }
        if (!alertText2.isEmpty()) {
            canvas.drawText(alertText2, cx, y2, alertText2Paint);
        }
    }

    /** Lead marker at model height ~1.32 m (flowpilot onroad lead). */
    private void drawLead(Canvas canvas) {
        ModelLongParse.Out ml = this.modelLong;
        if (ml == null || !ml.ok) {
            return;
        }
        ModelLongParse.Lead lead = ml.lead0.prob >= ml.lead1.prob ? ml.lead0 : ml.lead1;
        if (ml.lead2 != null && ml.lead2.prob > lead.prob) {
            lead = ml.lead2;
        }
        // Always draw candidate when d looks like a car; dim if !valid (prob head may be dead).
        if (lead.x[0] < 2f || lead.x[0] > 120f) {
            return;
        }
        if (!projectEgo(lead.x[0], lead.y[0], 1.32f)) {
            return;
        }
        float s = Math.max(18f, 900f / Math.max(lead.x[0], 4f));
        int alpha = lead.prob >= 0.4f ? 255 : 120;
        leadPaint.setAlpha(alpha);
        canvas.drawCircle(mapPt[0], mapPt[1], s, leadPaint);
        String hud = String.format(java.util.Locale.US, "LEAD %.0fm  v=%.1f  p=%.2g%s",
                lead.x[0], lead.v[0], lead.prob, lead.prob >= 0.4f ? "" : " (raw)");
        canvas.drawText(hud, 16f, getHeight() - 24f, leadTextPaint);
    }

    /** ONNX session.run + prep + e2e (capture → infer done). Top-right. */
    private void drawLatencyHud(Canvas canvas, LaneLines ll) {
        float inferMs = ll.inferDurationMs;
        float prepMs = ll.prepDurationMs;
        float e2eMs = 0f;
        if (ll.captureTimestampMs > 0 && ll.inferTimestampMs > ll.captureTimestampMs) {
            e2eMs = (float) (ll.inferTimestampMs - ll.captureTimestampMs);
        }
        String line1 = String.format(java.util.Locale.US, "infer %4.0f  prep %4.0f", inferMs, prepMs);
        String line2 = String.format(java.util.Locale.US, "e2e   %4.0f ms", e2eMs);

        float pad = 10f;
        textPaint.setTextSize(24f);
        textPaint.setTypeface(Typeface.MONOSPACE);
        float tw = Math.max(textPaint.measureText(line1), textPaint.measureText(line2));
        float x = getWidth() - tw - pad - 12f;
        float y = 40f;
        float h = 58f;
        canvas.drawRoundRect(x - pad, y - 28f, x + tw + pad, y - 28f + h, 8f, 8f, textBgPaint);
        textPaint.setColor(Color.rgb(120, 220, 255));
        canvas.drawText(line1, x, y, textPaint);
        textPaint.setColor(Color.rgb(255, 200, 80));
        canvas.drawText(line2, x, y + 28f, textPaint);
    }

    private void drawPurePursuit(Canvas canvas) {
        final float raX = -waypointShift;
        final float raY = 0f;
        final float ld = Math.max(0.5f, ppLookaheadM);


        path.reset();
        boolean started = false;
        final int nCirc = 64;
        for (int i = 0; i <= nCirc; i++) {
            double th = 2.0 * Math.PI * i / nCirc;
            float x = raX + ld * (float) Math.cos(th);
            float y = raY + ld * (float) Math.sin(th);
            if (!projectEgo(x, y, 0.3f)) {
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
        if (started) {
            canvas.drawPath(path, ppLdPaint);
        }


        float kappa = ppCurvature;
        float arcLen = Math.min(Math.max(ld * 1.5f, 15f), 50f);
        path.reset();
        started = false;
        final int nArc = 48;
        for (int i = 0; i < nArc; i++) {
            float s = arcLen * i / (nArc - 1);
            float ax;
            float ay;
            if (Math.abs(kappa) < 1e-6f) {
                ax = raX + s;
                ay = raY;
            } else {
                ax = raX + (float) (Math.sin(kappa * s) / kappa);
                ay = raY + (float) ((1.0 - Math.cos(kappa * s)) / kappa);
            }
            if (!projectEgo(ax, ay, 0.3f)) {
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
        if (started) {
            canvas.drawPath(path, ppArcPaint);
        }


        if (projectEgo(raX, raY, -5f)) {
            float px = mapPt[0];
            float py = mapPt[1];
            float r = 10f;
            canvas.drawLine(px - r, py - r, px + r, py + r, ppLdPaint);
            canvas.drawLine(px - r, py + r, px + r, py - r, ppLdPaint);
        }


        if (ppHasTarget && projectEgo(ppTargetX, ppTargetY, 0.3f)) {
            float tx = mapPt[0];
            float ty = mapPt[1];
            if (projectEgo(raX, raY, -5f)) {
                canvas.drawLine(mapPt[0], mapPt[1], tx, ty, ppRayPaint);
            }
            canvas.drawCircle(tx, ty, 10f, ppTargetPaint);
            ppTargetPaint.setStyle(Paint.Style.STROKE);
            ppTargetPaint.setStrokeWidth(2f);
            ppTargetPaint.setColor(Color.WHITE);
            canvas.drawCircle(tx, ty, 12f, ppTargetPaint);
            ppTargetPaint.setStyle(Paint.Style.FILL);
            ppTargetPaint.setColor(Color.CYAN);
        }
    }

    private void drawSteeringHud(Canvas canvas) {
        float radius = Math.min(56f, getWidth() * 0.07f);
        float cxHud = getWidth() - radius - 24f;
        float cyHud = getHeight() - radius - 72f;

        canvas.drawCircle(cxHud, cyHud, radius + 8f, hudFillPaint);
        canvas.drawCircle(cxHud, cyHud, radius, hudPaint);
        canvas.drawCircle(cxHud, cyHud, radius * 0.35f, hudPaint);


        float wheelDeg = (float) Math.toDegrees(ppSteerRad) * steerRatio;
        wheelDeg = Math.max(-120f, Math.min(120f, wheelDeg));
        double ang = Math.toRadians(wheelDeg);

        for (float a0 : new float[]{90f, 210f, 330f}) {
            double a = Math.toRadians(a0);
            float px = radius * 0.92f * (float) Math.cos(a);
            float py = -radius * 0.92f * (float) Math.sin(a);
            float[] p1 = rotateHud(px, py, ang);
            canvas.drawLine(cxHud, cyHud, cxHud + p1[0], cyHud + p1[1], hudPaint);
        }

        float[] top = rotateHud(0f, -radius * 0.85f, ang);
        Paint hub = new Paint(Paint.ANTI_ALIAS_FLAG);
        hub.setColor(Color.rgb(255, 200, 0));
        hub.setStyle(Paint.Style.FILL);
        canvas.drawCircle(cxHud, cyHud, 6f, hub);
        hub.setColor(Color.RED);
        canvas.drawCircle(cxHud + top[0], cyHud + top[1], 6f, hub);

        textPaint.setTextSize(26f);
        textPaint.setColor(Color.rgb(255, 200, 0));
        String road = String.format("%+.1f° road", Math.toDegrees(ppSteerRad));
        String sw = String.format("SW %+.0f°", wheelDeg);
        canvas.drawText(road, cxHud - radius, cyHud + radius + 28f, textPaint);
        textPaint.setColor(Color.LTGRAY);
        canvas.drawText(sw, cxHud - radius, cyHud + radius + 54f, textPaint);


        if (steerValid) {
            float barW = 14f;
            float barH = radius * 2f;
            float barX = cxHud - radius - 28f;
            float barY = cyHud - radius;
            Paint barBg = new Paint(Paint.ANTI_ALIAS_FLAG);
            barBg.setColor(Color.argb(160, 40, 40, 40));
            canvas.drawRect(barX, barY, barX + barW, barY + barH, barBg);
            float mid = barY + barH * 0.5f;
            float frac = Math.max(-1f, Math.min(1f, torqueCnm / MAX_TORQUE_CNM));
            Paint barFg = new Paint(Paint.ANTI_ALIAS_FLAG);
            barFg.setColor(steerEnabled ? Color.rgb(0, 220, 120) : Color.rgb(180, 180, 80));
            if (frac >= 0) {
                canvas.drawRect(barX, mid - frac * barH * 0.5f, barX + barW, mid, barFg);
            } else {
                canvas.drawRect(barX, mid, barX + barW, mid - frac * barH * 0.5f, barFg);
            }
            Paint midLine = new Paint(Paint.ANTI_ALIAS_FLAG);
            midLine.setColor(Color.WHITE);
            midLine.setStrokeWidth(2f);
            canvas.drawLine(barX - 2f, mid, barX + barW + 2f, mid, midLine);

            textPaint.setTextSize(22f);
            textPaint.setColor(steerEnabled ? Color.rgb(0, 220, 120) : Color.LTGRAY);
            canvas.drawText(
                    String.format("%s %d cNm", steerEnabled ? "TQ" : "off", torqueCnm),
                    barX - 8f,
                    barY - 8f,
                    textPaint);
        }
    }

    private static float[] rotateHud(float px, float py, double ang) {
        double c = Math.cos(ang);
        double s = Math.sin(ang);

        return new float[]{(float) (c * px + s * py), (float) (-s * px + c * py)};
    }

    private void drawPpStatus(Canvas canvas) {
        float kappa = ppCurvature;
        String curv;
        if (Math.abs(kappa) < 1e-6f) {
            curv = "κ=0  R=∞";
        } else {
            curv = String.format("κ=%.4f/m  R=%.1fm", kappa, 1.0 / Math.abs(kappa));
        }
        String line1 = String.format(
                "PP Ld=%.1fm  δ=%+.1f°  %s  %s",
                ppLookaheadM,
                Math.toDegrees(ppSteerRad),
                curv,
                ppHasTarget ? "" : "(no target)");
        String line2 = ppStatus.isEmpty() ? "magenta=arc  orange=Ld  cyan=target" : ("status=" + ppStatus);

        textPaint.setTextSize(26f);
        textPaint.setColor(Color.rgb(255, 165, 0));
        float pad = 8f;
        float x = 12f;
        float y = 40f;
        float w = Math.max(textPaint.measureText(line1), textPaint.measureText(line2)) + pad * 2;
        canvas.drawRect(x - pad, y - 28f, x + w, y + 36f, textBgPaint);
        canvas.drawText(line1, x, y, textPaint);
        textPaint.setTextSize(22f);
        textPaint.setColor(Color.rgb(200, 150, 200));
        canvas.drawText(line2, x, y + 28f, textPaint);
    }

    private void drawHcaStatus(Canvas canvas) {
        textPaint.setTextSize(24f);
        boolean ok = hcaStatus.startsWith("HCA ok");
        textPaint.setColor(ok ? Color.rgb(0, 220, 120) : Color.rgb(255, 120, 80));
        float pad = 8f;
        float x = 12f;
        float y = ppValid ? 108f : 40f;
        float w = textPaint.measureText(hcaStatus) + pad * 2;
        canvas.drawRect(x - pad, y - 26f, x + w, y + 10f, textBgPaint);
        canvas.drawText(hcaStatus, x, y, textPaint);
    }


    private void drawPolylineXYZ(Canvas canvas, float[] xs, float[] ys, float[] zs,
                                 Paint paint, float xMin, boolean pathLift) {
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
            if (!projectDevice(X, ys[i], Z, xMin, pathLift)) {
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
     * Remap device (X,Y,Z)→(Y,Z,X) then apply Rt. Device: X fwd, Y right+, Z up.
     * Rt matches warp: V·R(rpy)·V⁻¹ with R from {@link ModelCalibWarp#rotFromEuler}.
     */
    private boolean projectDevice(float X, float Y, float Z, float xMin, boolean pathLift) {
        if (X < xMin || !Float.isFinite(X) || !Float.isFinite(Y) || !Float.isFinite(Z)) {
            return false;
        }
        float camX = Y;
        float camY = Z + (pathLift ? PATH_LIFT_M : 0f);
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

    /** PP / HUD points at ~camera height in device-Z (like Draw lead at 1.32). */
    private boolean projectEgo(float X, float Y, float xMin) {
        return projectDevice(X, Y, cameraHeight, xMin, false);
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
