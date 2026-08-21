package adas.app.vision.overlay;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

/** Status frame, and the band that carries an alert. */
public final class StatusFramePainter {
    /** Border thickness, and the height of the band that replaces it when there is something to say. */
    private static final float BORDER_PX = 14f;
    private static final float BAND_PX = 300f;

    private final Paint borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint bandPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint text1Paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint text2Paint = new Paint(Paint.ANTI_ALIAS_FLAG);

    public StatusFramePainter() {
        borderPaint.setStyle(Paint.Style.FILL);
        // The band is deliberately more transparent than a thin frame would be (0.40 against 0.71):
        // it covers a third of the screen, so it has to let the road through. Visibility comes from
        // the area, not from the opacity.
        bandPaint.setStyle(Paint.Style.FILL);
        text1Paint.setColor(Color.WHITE);
        text1Paint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        text1Paint.setTextAlign(Paint.Align.CENTER);
        text2Paint.setColor(Color.WHITE);
        text2Paint.setTypeface(Typeface.DEFAULT);
        text2Paint.setTextAlign(Paint.Align.CENTER);
    }

    /** Text sizes for this screen; called on resize so nothing is fixed in raw pixels. */
    public void onUiScale(float ui) {
        text1Paint.setTextSize(52f * ui);
        text1Paint.setShadowLayer(6f * ui, 0f, 2f * ui, Color.argb(180, 0, 0, 0));
        text2Paint.setTextSize(30f * ui);
    }

    /**
     * \param rgb Frame colour, packed RGB — the state, decided by the view.
     * \param line1 Alert headline; empty when no alert is active.
     * \param line2 Alert detail; empty allowed.
     */
    public void draw(Canvas canvas, int w, int h, float ui, int rgb, String line1, String line2) {
        if (w <= 0 || h <= 0) {
            return;
        }
        final float border = BORDER_PX * ui;
        borderPaint.setColor(withAlpha(rgb, 150));
        canvas.drawRect(0, 0, w, border, borderPaint);
        canvas.drawRect(0, h - border, w, h, borderPaint);
        canvas.drawRect(0, border, border, h - border, borderPaint);
        canvas.drawRect(w - border, border, w, h - border, borderPaint);

        if (line1.isEmpty() && line2.isEmpty()) {
            return;
        }
        final float band = Math.min(BAND_PX * ui, h * 0.45f);
        bandPaint.setColor(withAlpha(rgb, 102));
        canvas.drawRect(0, h - band, w, h, bandPaint);

        final float cx = w * 0.5f;
        final float baseline1 = h - band + band * 0.44f;
        if (!line1.isEmpty()) {
            canvas.drawText(line1, cx, baseline1, text1Paint);
        }
        if (!line2.isEmpty()) {
            canvas.drawText(line2, cx, baseline1 + 44f * ui, text2Paint);
        }
    }

    private static int withAlpha(int rgb, int alpha) {
        return Color.argb(alpha, Color.red(rgb), Color.green(rgb), Color.blue(rgb));
    }
}
