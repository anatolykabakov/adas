package adas.app.ui;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.widget.TextView;

import java.io.File;
import java.io.FileWriter;
import java.util.Arrays;
import java.util.Locale;

import adas.app.vision.TrafficYoloRunner;

/**
 * On-device YOLO timing bench. Launch:
 *   adb shell am start -n adas.app/.TrafficYoloBenchActivity --ei iters 40 --ei warmup 5
 * Writes /sdcard/adas_models/traffic_yolo_bench.txt and logcat tag TrafficYoloBench.
 */
public final class TrafficYoloBenchActivity extends Activity {
    private static final String TAG = "TrafficYoloBench";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        TextView tv = new TextView(this);
        tv.setTextSize(16f);
        tv.setPadding(32, 48, 32, 32);
        tv.setText("Traffic YOLO bench…");
        setContentView(tv);

        final int iters = Math.max(1, getIntent().getIntExtra("iters", 40));
        final int warmup = Math.max(0, getIntent().getIntExtra("warmup", 5));
        final int srcW = getIntent().getIntExtra("width", 1280);
        final int srcH = getIntent().getIntExtra("height", 720);
        final String ep = getIntent().getStringExtra("ep");
        final String epPrefer = ep != null ? ep : "auto";

        new Thread(() -> {
            String report;
            try {
                report = runBench(iters, warmup, srcW, srcH, epPrefer);
            } catch (Throwable t) {
                report = "FAIL: " + t;
                Log.e(TAG, "bench failed", t);
            }
            final String out = report;
            runOnUiThread(() -> tv.setText(out));
        }, "TrafficYoloBench").start();
    }

    private String runBench(int iters, int warmup, int srcW, int srcH, String epPrefer) throws Exception {
        long tLoad0 = SystemClock.elapsedRealtime();
        TrafficYoloRunner runner = new TrafficYoloRunner(getApplicationContext(), epPrefer);
        long loadMs = SystemClock.elapsedRealtime() - tLoad0;

        Bitmap bmp = syntheticFrame(srcW, srcH);
        try {
            for (int i = 0; i < warmup; i++) {
                runner.runBitmap(bmp);
            }

            int[] total = new int[iters];
            int[] prep = new int[iters];
            int[] ort = new int[iters];
            int[] decode = new int[iters];
            int nDets = 0;
            for (int i = 0; i < iters; i++) {
                TrafficYoloRunner.Result r = runner.runBitmap(bmp);
                total[i] = r.inferMs;
                prep[i] = r.prepMs;
                ort[i] = r.ortMs;
                decode[i] = r.decodeMs;
                nDets = r.dets != null ? r.dets.size() : 0;
                Log.i(TAG, String.format(Locale.US,
                        "iter %02d/%d total=%d prep=%d ort=%d decode=%d dets=%d",
                        i + 1, iters, r.inferMs, r.prepMs, r.ortMs, r.decodeMs, nDets));
            }

            StringBuilder sb = new StringBuilder();
            sb.append("TrafficYoloBench\n");
            sb.append(String.format(Locale.US, "model=%s ep=%s load_ms=%d\n",
                    runner.modelName(), runner.epName(), loadMs));
            sb.append(String.format(Locale.US, "input=%dx%d letterbox=%d warmup=%d iters=%d last_dets=%d\n",
                    srcW, srcH, runner.inputSize(), warmup, iters, nDets));
            sb.append(statsLine("total_ms", total));
            sb.append(statsLine("prep_ms", prep));
            sb.append(statsLine("ort_ms", ort));
            sb.append(statsLine("decode_ms", decode));
            sb.append("DONE\n");
            String report = sb.toString();
            Log.i(TAG, "SUMMARY\n" + report);
            writeReport(report);
            return report;
        } finally {
            bmp.recycle();
            runner.close();
        }
    }

    private static Bitmap syntheticFrame(int w, int h) {
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(bmp);
        c.drawColor(Color.rgb(40, 80, 40));
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
        // Fake traffic light housing (bright colors → HSV path exercises decode).
        p.setColor(Color.rgb(20, 20, 20));
        c.drawRect(w * 0.45f, h * 0.15f, w * 0.55f, h * 0.45f, p);
        p.setColor(Color.RED);
        c.drawCircle(w * 0.5f, h * 0.22f, Math.min(w, h) * 0.03f, p);
        p.setColor(Color.YELLOW);
        c.drawCircle(w * 0.5f, h * 0.30f, Math.min(w, h) * 0.03f, p);
        p.setColor(Color.GREEN);
        c.drawCircle(w * 0.5f, h * 0.38f, Math.min(w, h) * 0.03f, p);
        return bmp;
    }

    private static String statsLine(String name, int[] v) {
        int[] s = v.clone();
        Arrays.sort(s);
        double sum = 0;
        for (int x : s) {
            sum += x;
        }
        int n = s.length;
        int p50 = s[n / 2];
        int p95 = s[Math.min(n - 1, (int) Math.ceil(0.95 * n) - 1)];
        return String.format(Locale.US,
                "%s: min=%d med=%d p95=%d max=%d mean=%.1f\n",
                name, s[0], p50, p95, s[n - 1], sum / n);
    }

    private void writeReport(String report) {
        try {
            File dir = new File("/sdcard/adas_models");
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            File f = new File(dir, "traffic_yolo_bench.txt");
            try (FileWriter w = new FileWriter(f, false)) {
                w.write(report);
            }
            Log.i(TAG, "wrote " + f.getAbsolutePath());
        } catch (Exception e) {
            Log.w(TAG, "could not write report file", e);
        }
    }
}
