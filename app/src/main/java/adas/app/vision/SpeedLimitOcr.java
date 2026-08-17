package adas.app.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

/**
 * Fast digit-only OCR for RU circular speed-limit crops.
 * No ML Kit / Tesseract — template match of 0–9 (~1–5 ms per crop).
 */
public final class SpeedLimitOcr {
    private static final int TH = 48;
    private static final int DIGIT_W = 28;
    private static final int DIGIT_H = 40;
    private static final Bitmap[] TEMPLATES = buildTemplates();

    private SpeedLimitOcr() {}

    /**
     * @return km/h in {5..150 step 5}, or 0 if undecided
     */
    public static int readKmh(Bitmap src, int x1, int y1, int x2, int y2) {
        if (src == null) {
            return 0;
        }
        int w = Math.max(1, x2 - x1);
        int h = Math.max(1, y2 - y1);
        if (w < 12 || h < 12) {
            return 0;
        }
        // Digits sit in the white center of the round sign — crop inner ~55%.
        int ix1 = x1 + (int) (w * 0.22f);
        int iy1 = y1 + (int) (h * 0.22f);
        int ix2 = x2 - (int) (w * 0.22f);
        int iy2 = y2 - (int) (h * 0.22f);
        if (ix2 - ix1 < 8 || iy2 - iy1 < 8) {
            ix1 = x1;
            iy1 = y1;
            ix2 = x2;
            iy2 = y2;
        }
        ix1 = clamp(ix1, 0, src.getWidth() - 1);
        iy1 = clamp(iy1, 0, src.getHeight() - 1);
        ix2 = clamp(ix2, ix1 + 1, src.getWidth());
        iy2 = clamp(iy2, iy1 + 1, src.getHeight());

        Bitmap crop = Bitmap.createBitmap(src, ix1, iy1, ix2 - ix1, iy2 - iy1);
        try {
            int tw = Math.max(16, crop.getWidth() * TH / Math.max(1, crop.getHeight()));
            Bitmap scaled = Bitmap.createScaledBitmap(crop, tw, TH, true);
            try {
                boolean[][] ink = toInk(scaled);
                int[] cols = projection(ink);
                java.util.List<int[]> spans = digitSpans(cols);
                if (spans.isEmpty() || spans.size() > 3) {
                    return 0;
                }
                StringBuilder sb = new StringBuilder();
                for (int[] sp : spans) {
                    int d = matchDigit(ink, sp[0], sp[1]);
                    if (d < 0) {
                        return 0;
                    }
                    sb.append(d);
                }
                int v = Integer.parseInt(sb.toString());
                if (v >= 5 && v <= 150 && v % 5 == 0) {
                    return v;
                }
                return 0;
            } finally {
                if (scaled != crop) {
                    scaled.recycle();
                }
            }
        } finally {
            crop.recycle();
        }
    }

    private static boolean[][] toInk(Bitmap bmp) {
        int w = bmp.getWidth();
        int h = bmp.getHeight();
        int[] px = new int[w * h];
        bmp.getPixels(px, 0, w, 0, 0, w, h);
        long sum = 0;
        int[] gray = new int[w * h];
        for (int i = 0; i < gray.length; i++) {
            int p = px[i];
            int g = (((p >> 16) & 255) * 30 + ((p >> 8) & 255) * 59 + (p & 255) * 11) / 100;
            gray[i] = g;
            sum += g;
        }
        int thr = (int) (sum / gray.length);
        // Prefer dark digits on bright plate: ink = below threshold.
        boolean[][] ink = new boolean[h][w];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                ink[y][x] = gray[y * w + x] < thr - 8;
            }
        }
        return ink;
    }

    private static int[] projection(boolean[][] ink) {
        int h = ink.length;
        int w = ink[0].length;
        int[] cols = new int[w];
        for (int x = 0; x < w; x++) {
            int c = 0;
            for (int y = 0; y < h; y++) {
                if (ink[y][x]) {
                    c++;
                }
            }
            cols[x] = c;
        }
        return cols;
    }

    private static java.util.List<int[]> digitSpans(int[] cols) {
        java.util.List<int[]> out = new java.util.ArrayList<>();
        int minInk = Math.max(2, (TH * 8) / 100);
        int i = 0;
        while (i < cols.length) {
            while (i < cols.length && cols[i] < minInk) {
                i++;
            }
            if (i >= cols.length) {
                break;
            }
            int a = i;
            while (i < cols.length && cols[i] >= minInk) {
                i++;
            }
            int b = i;
            if (b - a >= 3 && b - a <= cols.length / 2 + 4) {
                out.add(new int[]{a, b});
            }
        }
        return out;
    }

    private static int matchDigit(boolean[][] ink, int x0, int x1) {
        int h = ink.length;
        int bw = Math.max(1, x1 - x0);
        Bitmap dig = Bitmap.createBitmap(DIGIT_W, DIGIT_H, Bitmap.Config.ARGB_8888);
        // rasterize crop region into DIGIT_W x DIGIT_H
        int[] px = new int[DIGIT_W * DIGIT_H];
        for (int y = 0; y < DIGIT_H; y++) {
            int sy = y * h / DIGIT_H;
            for (int x = 0; x < DIGIT_W; x++) {
                int sx = x0 + x * bw / DIGIT_W;
                boolean on = sy < h && sx < ink[0].length && ink[sy][sx];
                px[y * DIGIT_W + x] = on ? 0xff000000 : 0xffffffff;
            }
        }
        dig.setPixels(px, 0, DIGIT_W, 0, 0, DIGIT_W, DIGIT_H);
        int best = -1;
        float bestScore = 0.55f;
        for (int d = 0; d <= 9; d++) {
            float s = corr(dig, TEMPLATES[d]);
            if (s > bestScore) {
                bestScore = s;
                best = d;
            }
        }
        dig.recycle();
        return best;
    }

    private static float corr(Bitmap a, Bitmap b) {
        int n = DIGIT_W * DIGIT_H;
        int[] pa = new int[n];
        int[] pb = new int[n];
        a.getPixels(pa, 0, DIGIT_W, 0, 0, DIGIT_W, DIGIT_H);
        b.getPixels(pb, 0, DIGIT_W, 0, 0, DIGIT_W, DIGIT_H);
        int match = 0;
        for (int i = 0; i < n; i++) {
            boolean ia = (pa[i] & 0xff) < 128;
            boolean ib = (pb[i] & 0xff) < 128;
            if (ia == ib) {
                match++;
            }
        }
        return match / (float) n;
    }

    private static Bitmap[] buildTemplates() {
        Bitmap[] t = new Bitmap[10];
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
        p.setColor(Color.BLACK);
        p.setTextSize(36f);
        p.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        p.setTextAlign(Paint.Align.CENTER);
        Paint.FontMetrics fm = p.getFontMetrics();
        float cy = DIGIT_H / 2f - (fm.ascent + fm.descent) / 2f;
        for (int d = 0; d <= 9; d++) {
            Bitmap b = Bitmap.createBitmap(DIGIT_W, DIGIT_H, Bitmap.Config.ARGB_8888);
            Canvas c = new Canvas(b);
            c.drawColor(Color.WHITE);
            c.drawText(String.valueOf(d), DIGIT_W / 2f, cy, p);
            t[d] = b;
        }
        return t;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
