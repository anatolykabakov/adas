package adas.app.vision.overlay;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;

import adas.app.vision.ModelLongParse;

/** The lead marker: a chevron of near-constant screen size, openpilot's shape and its arithmetic. */
public final class LeadPainter {
    /**
     * openpilot writes the chevron's size for its own 2160-wide screen; this carries the same share of
     * the width over to our 1280-wide reference, so the marker occupies the same fraction of the view
     * as it does on a comma three.
     */
    private static final float OP_REF_W = 1280f / 2160f;
    /** Distance under which the chevron starts filling red, and the closing speed that fills it. */
    private static final float LEAD_BUFF_M = 40f;
    private static final float SPEED_BUFF_MPS = 10f;
    /** Below this the model is not claiming a car; drawing it anyway invents traffic. */
    private static final float MIN_LEAD_PROB = 0.5f;

    private final Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path path = new Path();
    private final float[] pt = new float[2];

    public LeadPainter() {
        fillPaint.setStyle(Paint.Style.FILL);
        textPaint.setColor(Color.rgb(0, 220, 255));
        textPaint.setTypeface(Typeface.MONOSPACE);
        textBgPaint.setColor(Color.argb(120, 0, 0, 0));
        textBgPaint.setStyle(Paint.Style.FILL);
    }

    /**
     * \param ml Parsed longitudinal output; nothing is drawn unless it carries a confident lead.
     * \param frameDtMs Camera period, for the frame-spacing velocity correction (ModelLongParse).
     * \param egoValid / egoMps Own speed for the closing-speed fill; without it vRel is treated as 0.
     * \param cameraHeight The road plane below the camera, where the lead's feet are.
     */
    public void draw(Canvas canvas, ModelLongParse.Out ml, float frameDtMs, boolean egoValid,
                     float egoMps, float cameraHeight, float ui, int viewW, int viewH,
                     GroundProjector projector) {
        if (ml == null || !ml.ok) {
            return;
        }
        ModelLongParse.Lead lead = ml.lead0.prob >= ml.lead1.prob ? ml.lead0 : ml.lead1;
        if (ml.lead2 != null && ml.lead2.prob > lead.prob) {
            lead = ml.lead2;
        }
        if (lead.prob < MIN_LEAD_PROB || lead.x[0] < 2f || lead.x[0] > 120f) {
            return;
        }
        final float d = lead.x[0];
        // Where the lead is on screen: still a world projection, at the calibrated camera height.
        if (!projector.project(d, lead.y[0], cameraHeight, 1.0f, pt)) {
            return;
        }
        final float sz = Math.min(Math.max((25f * 30f) / (d / 3f + 30f), 15f), 30f)
                * 2.35f * OP_REF_W * ui;
        final float gx = sz / 5f;
        final float gy = sz / 10f;
        // Keep it on screen: x as openpilot clamps it, and y high enough that the body below the tip
        // and the label below the base both stay in the frame.
        final float x = Math.min(Math.max(pt[0], sz * 0.5f), viewW - sz * 0.5f);
        final float labelRoom = 34f * ui;
        final float y = Math.min(pt[1], viewH - sz - gy - labelRoom);

        path.reset();
        path.moveTo(x + sz * 1.35f + gx, y + sz + gy);
        path.lineTo(x, y - gy);
        path.lineTo(x - sz * 1.35f - gx, y + sz + gy);
        path.close();
        fillPaint.setColor(Color.rgb(218, 202, 37));
        canvas.drawPath(path, fillPaint);

        // vRel: the model's velocity needs the frame-spacing correction before it can be compared
        // with the wheel speed at all (ModelLongParse). Without an ego speed yet, treat it as steady.
        final float vLead = lead.v[0] * ModelLongParse.velocityScale(frameDtMs);
        final float vRel = egoValid ? vLead - egoMps : 0f;
        float fillAlpha = 0f;
        if (d < LEAD_BUFF_M) {
            fillAlpha = 255f * (1f - d / LEAD_BUFF_M);
            if (vRel < 0f) {
                fillAlpha += 255f * (-vRel / SPEED_BUFF_MPS);
            }
            fillAlpha = Math.min(fillAlpha, 255f);
        }
        path.reset();
        path.moveTo(x + sz * 1.25f, y + sz);
        path.lineTo(x, y);
        path.lineTo(x - sz * 1.25f, y + sz);
        path.close();
        fillPaint.setColor(Color.argb((int) fillAlpha, 201, 34, 49));
        canvas.drawPath(path, fillPaint);

        textPaint.setTextSize(26f * ui);
        final String label = String.format(java.util.Locale.US, "%.0f m   %.0f km/h", d, vLead * 3.6f);
        final float tw = textPaint.measureText(label);
        final float tx = Math.min(Math.max(x - tw * 0.5f, 8f * ui), viewW - tw - 8f * ui);
        // Under the base of the triangle: the rim reaches gy past it, so start below that.
        final float ty = y + sz + gy + 24f * ui;
        canvas.drawRect(tx - 6f * ui, ty - 22f * ui, tx + tw + 6f * ui, ty + 6f * ui, textBgPaint);
        canvas.drawText(label, tx, ty, textPaint);
    }
}
