package adas.app.ui;

import android.app.Activity;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.widget.TextView;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import adas.app.vision.ModelRunner;
import adas.app.vision.SupercomboOnnxRunner;
import adas.app.vision.SupercomboThneedRunner;
import adas.app.vision.YuvFrame;

/**
 * Which vision runners this phone can actually use, how fast, and whether they agree.
 *
 * <p>thneed is a recorded GPU run of the network — kernel sources, launch order, buffers — and the
 * driver of this phone compiles those sources at load. So the question for a new phone is not
 * "compile it" but "does it load here, how fast does it run, and does it give the same numbers".
 * That is what this measures. The OpenCL capabilities it also reports are there to explain a
 * failure, not to predict one: this device declares a 64-pixel row-pitch alignment yet happily
 * accepts images with a 128-byte pitch.
 *
 * <p>Two things are reported and both matter. Latency decides whether the phone is usable at all —
 * vision rate sets the setpoint step (see {@code docs/VISION_RATE.md}). The output signature decides
 * whether the fast path may be trusted: a replay that runs but produces different numbers is worse
 * than one that fails, because nothing announces it.
 *
 * <p>Both runners now carry the same network, so their signatures are directly comparable — and on
 * this device they agree to fp16 rounding. That comparison was impossible while the two paths ran
 * different model generations.
 *
 * <pre>
 * adb shell am start -n adas.app/adas.app.ui.SupercomboBenchActivity --ei iters 50 --ei warmup 5
 * adb shell cat /sdcard/adas_models/supercombo_bench.json
 * </pre>
 *
 * Driven by {@code scripts/tools/model_device_probe.py}, which reads the JSON and decides what to put
 * in {@code vision.model_runner}.
 */
public final class SupercomboBenchActivity extends Activity {
    private static final String TAG = "SupercomboBench";
    private static final String OUT = "/sdcard/adas_models/supercombo_bench.json";

    private static final int WIDTH = 1280;
    private static final int HEIGHT = 720;

    /** How many leading output values go into the signature, for a cheap eyeball comparison. */
    private static final int SIGNATURE_HEAD = 8;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        TextView tv = new TextView(this);
        tv.setTextSize(14f);
        tv.setPadding(32, 48, 32, 32);
        tv.setText("Supercombo bench…");
        setContentView(tv);

        final int iters = Math.max(1, getIntent().getIntExtra("iters", 50));
        final int warmup = Math.max(0, getIntent().getIntExtra("warmup", 5));
        // Rebuild the model with this GPU's compiler and write the result: --ez save true
        final boolean saveCompiled = getIntent().getBooleanExtra("save", false);

        new Thread(() -> {
            String report;
            try {
                report = run(iters, warmup, saveCompiled);
            } catch (Throwable t) {
                report = "{\"error\": \"" + String.valueOf(t.getMessage()).replace('"', '\'') + "\"}";
                Log.e(TAG, "bench failed", t);
            }
            final String out = report;
            try {
                // The directory may not exist at all: on a fresh phone nobody created it, and the
                // benchmark is the first thing run there. Without this the report was lost even though
                // the measurement itself succeeded.
                final File file = new File(OUT);
                final File dir = file.getParentFile();
                if (dir != null && !dir.isDirectory() && !dir.mkdirs()) {
                    Log.w(TAG, "could not create " + dir);
                }
                try (FileWriter w = new FileWriter(file)) {
                    w.write(out);
                }
            } catch (Exception e) {
                Log.e(TAG, "cannot write " + OUT, e);
            }
            Log.i(TAG, out);
            runOnUiThread(() -> tv.setText(out));
        }, "SupercomboBench").start();
    }

    private String run(int iters, int warmup, boolean saveCompiled) {
        final YuvFrame frame = syntheticFrame();
        final StringBuilder json = new StringBuilder("{\n");
        json.append("  \"device\": ").append(deviceJson()).append(",\n");
        // OpenCL capabilities — whether the fast path comes up on this phone at all depends on them.
        json.append("  \"opencl\": ").append(SupercomboThneedRunner.openClInfo()).append(",\n");
        json.append("  \"iters\": ").append(iters).append(",\n  \"runners\": [\n");

        final List<String> entries = new ArrayList<>();

        for (String which : new String[]{"onnx", "thneed"}) {
            ModelRunner runner = null;
            try {
                runner = "thneed".equals(which)
                        ? new SupercomboThneedRunner(getApplicationContext())
                        : new SupercomboOnnxRunner(getApplicationContext());
                runner.setCalib(0f, 0f, 0f, 930f, 930f, WIDTH / 2f, HEIGHT / 2f, WIDTH, HEIGHT);

                // Both need a previous frame before they return anything, so the first runs are not
                // measurements — they are the model filling its temporal buffer.
                for (int i = 0; i < Math.max(2, warmup); i++) {
                    runner.run(frame, i, SystemClock.elapsedRealtime());
                }

                final long[] us = new long[iters];
                // Preparation apart from the compute itself. Without that split a "runner median" is
                // the sum of two different jobs and it is unclear what to optimise: the frame warp
                // runs on the CPU and the network on the GPU, and their shares differ.
                final float[] prepMs = new float[iters];
                final float[] inferMs = new float[iters];
                SupercomboOnnxRunner.Result last = null;
                for (int i = 0; i < iters; i++) {
                    long t0 = SystemClock.elapsedRealtimeNanos();
                    SupercomboOnnxRunner.Result r = runner.run(frame, 100 + i, SystemClock.elapsedRealtime());
                    us[i] = (SystemClock.elapsedRealtimeNanos() - t0) / 1000;
                    if (r != null) {
                        last = r;
                        if (r.lanes != null) {
                            prepMs[i] = r.lanes.prepDurationMs;
                            inferMs[i] = r.lanes.inferDurationMs;
                        }
                    }
                }
                Arrays.sort(us);
                Arrays.sort(prepMs);
                Arrays.sort(inferMs);
                final float[] output = last != null && last.lanes != null ? last.lanes.modelOut : null;

                if (saveCompiled && "thneed".equals(which)) {
                    // The kernels were already built by the Adreno driver at load time; here we merely
                    // freeze them so the next start does not pay for building them again.
                    final String out = "/sdcard/adas_models/supercombo_adreno.thneed";
                    final boolean ok = SupercomboThneedRunner.saveCompiled(out, true);
                    Log.i(TAG, "saveCompiled(" + out + ") = " + ok);
                }

                entries.add(String.format(Locale.US,
                        "    {\"name\": \"%s\", \"ok\": true, \"median_ms\": %.2f, \"p95_ms\": %.2f, "
                                + "\"min_ms\": %.2f, \"prep_ms\": %.2f, \"infer_ms\": %.2f, \"signature\": %s}",
                        runner.name(), us[us.length / 2] / 1000.0, us[(int) (us.length * 0.95)] / 1000.0,
                        us[0] / 1000.0, prepMs[prepMs.length / 2], inferMs[inferMs.length / 2],
                        signatureJson(output)));
            } catch (Throwable t) {
                // Not a failure of the bench: a phone without an Adreno GPU cannot replay a thneed at
                // all, and saying so is the answer, not an error.
                entries.add(String.format(Locale.US, "    {\"name\": \"%s\", \"ok\": false, \"reason\": \"%s\"}",
                        which, String.valueOf(t.getMessage()).replace('"', '\'')));
                Log.w(TAG, which + " unavailable", t);
            } finally {
                if (runner != null) {
                    try {
                        runner.close();
                    } catch (Exception ignored) {
                    }
                }
            }
        }

        json.append(String.join(",\n", entries)).append("\n  ]\n}\n");
        return json.toString();
    }

    /**
     * A compact fingerprint of one model output: enough to tell "same numbers" from "ran but wrong".
     *
     * <p>Length, mean and spread catch a replay that produced zeros or noise; the leading values catch
     * a layout that shifted. Cheap enough to sit in a report and be diffed by eye.
     */
    private static String signatureJson(float[] out) {
        if (out == null || out.length == 0) {
            return "{\"len\": 0}";
        }
        double sum = 0.0;
        double sumsq = 0.0;
        for (float v : out) {
            sum += v;
            sumsq += (double) v * v;
        }
        final double mean = sum / out.length;
        final double var = Math.max(0.0, sumsq / out.length - mean * mean);
        final StringBuilder head = new StringBuilder("[");
        for (int i = 0; i < Math.min(SIGNATURE_HEAD, out.length); i++) {
            if (i > 0) {
                head.append(", ");
            }
            head.append(String.format(Locale.US, "%.5f", out[i]));
        }
        head.append(']');
        return String.format(Locale.US, "{\"len\": %d, \"mean\": %.5f, \"std\": %.5f, \"head\": %s}",
                out.length, mean, Math.sqrt(var), head);
    }

    private static String deviceJson() {
        return String.format(Locale.US,
                "{\"model\": \"%s\", \"device\": \"%s\", \"board\": \"%s\", \"soc\": \"%s %s\", "
                        + "\"abi\": \"%s\", \"sdk\": %d}",
                android.os.Build.MODEL, android.os.Build.DEVICE, android.os.Build.BOARD,
                android.os.Build.SOC_MANUFACTURER, android.os.Build.SOC_MODEL,
                android.os.Build.SUPPORTED_ABIS.length > 0 ? android.os.Build.SUPPORTED_ABIS[0] : "?",
                android.os.Build.VERSION.SDK_INT);
    }

    /**
     * A fixed synthetic frame — a diagonal gradient with a horizon.
     *
     * <p>Deliberately not a camera frame: the two runners must see byte-identical input for their
     * outputs to be comparable at all, and a live camera guarantees they never will.
     */
    private static YuvFrame syntheticFrame() {
        final byte[] y = new byte[WIDTH * HEIGHT];
        for (int row = 0; row < HEIGHT; row++) {
            for (int col = 0; col < WIDTH; col++) {
                int v = row < HEIGHT / 2 ? 140 + ((col * 40) / WIDTH) : 60 + (((row + col) * 60) / (WIDTH + HEIGHT));
                y[row * WIDTH + col] = (byte) v;
            }
        }
        final byte[] u = new byte[WIDTH * HEIGHT / 4];
        final byte[] v = new byte[WIDTH * HEIGHT / 4];
        Arrays.fill(u, (byte) 128);
        Arrays.fill(v, (byte) 128);
        return new YuvFrame(WIDTH, HEIGHT, y, u, v);
    }
}
