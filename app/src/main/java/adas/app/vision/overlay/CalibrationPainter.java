package adas.app.vision.overlay;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

/** The lens-calibration view: progress panel on top, detected board corners over the preview. */
public final class CalibrationPainter {
    private final Paint cornerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint panelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    /**
     * \param corners Flat x,y pairs in frame pixels; null when the board was not found this frame.
     * \param accepted True when the view was kept — green — rather than rejected as a near-duplicate.
     * \param kept / target Collection progress for the bar.
     * \param message One line of instruction or result; may be empty.
     */
    public void draw(Canvas canvas, int viewW, int viewH, float frameW, float frameH,
                     float[] corners, boolean accepted, int kept, int target, String message) {
        drawPanel(canvas, viewW, kept, target, message);
        if (corners == null || corners.length < 2) {
            return;
        }
        final float sx = viewW / frameW;
        final float sy = viewH / frameH;
        cornerPaint.setColor(accepted ? Color.GREEN : Color.rgb(255, 176, 0));
        cornerPaint.setStyle(Paint.Style.STROKE);
        cornerPaint.setStrokeWidth(3f);
        for (int i = 0; i + 3 < corners.length; i += 2) {
            canvas.drawLine(corners[i] * sx, corners[i + 1] * sy,
                    corners[i + 2] * sx, corners[i + 3] * sy, cornerPaint);
        }
        cornerPaint.setStyle(Paint.Style.FILL);
        for (int i = 0; i + 1 < corners.length; i += 2) {
            canvas.drawCircle(corners[i] * sx, corners[i + 1] * sy, 7f, cornerPaint);
        }
    }

    private void drawPanel(Canvas canvas, int viewW, int kept, int target, String message) {
        final float pad = 24f;
        final float barH = 18f;
        final float panelH = 132f;

        panelPaint.setColor(Color.argb(190, 0, 0, 0));
        canvas.drawRect(0f, 0f, viewW, panelH, panelPaint);

        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(40f);
        textPaint.setFakeBoldText(true);
        canvas.drawText(kept + " / " + target, pad, 50f, textPaint);

        textPaint.setTextSize(26f);
        textPaint.setFakeBoldText(false);
        if (!message.isEmpty()) {
            canvas.drawText(message, pad + 160f, 46f, textPaint);
        }

        // Progress bar: how many views have been accepted out of the number required.
        final float barY = panelH - pad - barH;
        panelPaint.setColor(Color.argb(120, 255, 255, 255));
        canvas.drawRect(pad, barY, viewW - pad, barY + barH, panelPaint);
        panelPaint.setColor(kept >= target ? Color.GREEN : Color.rgb(0, 170, 255));
        final float frac = Math.min(1f, kept / (float) target);
        canvas.drawRect(pad, barY, pad + (viewW - 2 * pad) * frac, barY + barH, panelPaint);
    }
}
