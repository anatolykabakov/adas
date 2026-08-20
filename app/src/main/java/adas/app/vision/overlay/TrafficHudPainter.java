package adas.app.vision.overlay;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import adas.app.vision.TrafficYoloRunner;

/** Everything the traffic detector puts on screen: detection boxes, the speed-limit disc, and the traffic light. */
public final class TrafficHudPainter {
    /**
     * Radius of the traffic-light disc as a share of the view width — 5.5 % is about 129 px on the 7T,
     * against the 11 px lamp it replaces.
     */
    private static final float TFL_DISC_FRACTION = 0.055f;

    private final Paint discPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint boxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private volatile java.util.List<TrafficYoloRunner.Det> dets = java.util.Collections.emptyList();
    private volatile int tflColor; // TrafficLightColor number
    private volatile float tflConf;

    public TrafficHudPainter() {
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(3f);
        boxPaint.setColor(Color.rgb(255, 200, 40));
        discPaint.setStyle(Paint.Style.FILL);
        textPaint.setColor(Color.rgb(255, 200, 0));
        textPaint.setTypeface(Typeface.MONOSPACE);
    }

    public void setDets(java.util.List<TrafficYoloRunner.Det> dets) {
        this.dets = dets != null ? dets : java.util.Collections.emptyList();
    }

    public void setVision(int tflColor, float tflConf) {
        this.tflColor = tflColor;
        this.tflConf = tflConf;
    }

    public void draw(Canvas canvas, int w, int h, float ui) {
        if (w <= 0 || h <= 0) {
            return;
        }
        drawDetBoxes(canvas, w, h, ui);
        drawTrafficLight(canvas, w, h, ui);
    }

    /** Detection boxes (normalized → view), coloured by what was found. */
    private void drawDetBoxes(Canvas canvas, int w, int h, float ui) {
        final java.util.List<TrafficYoloRunner.Det> dets = this.dets;
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
            boxPaint.setStrokeWidth(3f * ui);
            boxPaint.setColor(col);
            canvas.drawRect(l, t, r, b, boxPaint);
            textPaint.setTextSize(22f * ui);
            textPaint.setColor(col);
            String tag = d.label;
            if (d.speedLimitKmh > 0) {
                tag = tag + " " + d.speedLimitKmh + (d.speedFromOcr ? " OCR" : "");
            }
            canvas.drawText(tag + String.format(java.util.Locale.US, " %.2f", d.conf),
                    l + 4f * ui, Math.max(24f * ui, t - 6f * ui), textPaint);
        }
    }

    /** The light, as one big disc a driver can read without looking for it. */
    private void drawTrafficLight(Canvas canvas, int w, int h, float ui) {
        final int colour;
        switch (tflColor) {
            case 1: colour = Color.rgb(230, 40, 40); break;      // red
            case 2: colour = Color.rgb(240, 200, 40); break;     // yellow
            case 3: colour = Color.rgb(40, 210, 90); break;      // green
            default: return;                                     // unknown / off: say nothing
        }
        final float r = w * TFL_DISC_FRACTION;
        final float cx = r + 28f * ui;
        final float cy = h * 0.5f;

        discPaint.setColor(Color.argb(90, 0, 0, 0));
        canvas.drawCircle(cx, cy, r + 6f * ui, discPaint);       // a ring, for a bright sky
        discPaint.setColor(colour);
        canvas.drawCircle(cx, cy, r, discPaint);

        if (tflConf > 0.01f) {
            textPaint.setTextSize(24f * ui);
            textPaint.setColor(Color.WHITE);
            textPaint.setTypeface(Typeface.DEFAULT_BOLD);
            final String pct = String.format(java.util.Locale.US, "%.0f%%", tflConf * 100f);
            canvas.drawText(pct, cx - textPaint.measureText(pct) * 0.5f, cy + r + 30f * ui, textPaint);
            textPaint.setTypeface(Typeface.MONOSPACE);
        }
    }
}
