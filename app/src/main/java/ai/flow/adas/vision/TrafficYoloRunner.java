package ai.flow.adas.vision;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import ai.flow.adas.AdasConfig;
import ai.onnxruntime.NodeInfo;
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtLoggingLevel;
import ai.onnxruntime.OrtSession;
import ai.onnxruntime.TensorInfo;
import ai.onnxruntime.ValueInfo;
import ai.onnxruntime.providers.NNAPIFlags;

/**
 * Low-freq YOLOv8 ONNX detector for traffic lights + optional RU speed signs.
 * Expects Ultralytics YOLOv8 export: output [1, 4+nc, N] or [1, N, 4+nc].
 * Input size is read from the ONNX graph (320 / 256 / 192).
 */
public final class TrafficYoloRunner {
    private static final String TAG = "TrafficYolo";
    /** Default / docs; actual size = {@link #inputSize()}. */
    public static final int INPUT = 256;
    private static final float CONF_THRES = 0.35f;
    private static final float IOU_THRES = 0.45f;
    private static final Pattern SPEED_PAT = Pattern.compile(
            "(?:speed[_-]?|3[_./]?24[_./]?|limit[_-]?|ограничение\\s*скорости\\s*)(\\d{2,3})|\\b(\\d{2,3})\\b",
            Pattern.CASE_INSENSITIVE | Pattern.UNICODE_CASE);

    private final OrtEnvironment env;
    private final OrtSession session;
    private final String inputName;
    private final String[] labels;
    private final int inputSize;
    private final float[] inputBuf;
    private final int[] pixels;
    private final String modelName;
    private final String epName;
    /** Feature flags: which class families to keep after decode. */
    private final boolean enableSigns;
    private final boolean enableLights;

    public static final class Det {
        public String label;
        public float conf;
        public float x1, y1, x2, y2; // normalized
        public int speedLimitKmh;
        public int tflColor; // 0 unk 1 red 2 yellow 3 green 4 off
        public boolean speedFromOcr;
    }

    public static final class Result {
        public final List<Det> dets;
        /** Total wall time (prep + ORT + decode + ocr), ms. */
        public final int inferMs;
        public final int prepMs;
        public final int ortMs;
        public final int decodeMs;
        public final int ocrMs;
        public final String model;
        public final String ep;
        public final int inputSize;

        public Result(List<Det> dets, int inferMs, String model) {
            this(dets, inferMs, inferMs, 0, 0, 0, model, "", 0);
        }

        public Result(List<Det> dets, int inferMs, int prepMs, int ortMs, int decodeMs, int ocrMs,
                      String model, String ep, int inputSize) {
            this.dets = dets;
            this.inferMs = inferMs;
            this.prepMs = prepMs;
            this.ortMs = ortMs;
            this.decodeMs = decodeMs;
            this.ocrMs = ocrMs;
            this.model = model;
            this.ep = ep;
            this.inputSize = inputSize;
        }
    }

    public TrafficYoloRunner(Context context) throws Exception {
        this(context, "auto",
                AdasConfig.visionTrafficSignsEnabled(context),
                AdasConfig.visionTrafficLightsEnabled(context));
    }

    /**
     * @param epPrefer auto | nnapi | xnnpack | cpu
     */
    public TrafficYoloRunner(Context context, String epPrefer) throws Exception {
        this(context, epPrefer,
                AdasConfig.visionTrafficSignsEnabled(context),
                AdasConfig.visionTrafficLightsEnabled(context));
    }

    /**
     * @param epPrefer auto | nnapi | xnnpack | cpu
     * @param enableSigns keep sign / speed / VRU classes
     * @param enableLights keep traffic-light classes (+ HSV color)
     */
    public TrafficYoloRunner(Context context, String epPrefer,
                             boolean enableSigns, boolean enableLights) throws Exception {
        this.enableSigns = enableSigns;
        this.enableLights = enableLights;
        env = OrtEnvironment.getEnvironment();
        modelName = AdasConfig.trafficYoloAsset(context);
        File model = resolveModelFile(context, modelName);
        Log.i(TAG, "Loading " + model.getAbsolutePath() + " (" + model.length() + " bytes)"
                + " providers=" + env.getAvailableProviders());
        String prefer = epPrefer == null || epPrefer.isEmpty() ? "auto" : epPrefer.toLowerCase(Locale.US);
        SessionCreated sc = createSession(model.getAbsolutePath(), prefer);
        session = sc.session;
        epName = sc.epName;
        inputName = session.getInputNames().iterator().next();
        inputSize = readInputSize(session, inputName);
        inputBuf = new float[3 * inputSize * inputSize];
        pixels = new int[inputSize * inputSize];
        labels = loadLabels(context);
        Log.i(TAG, "ready ep=" + epName + " in=" + inputName + " size=" + inputSize
                + " labels=" + labels.length
                + " signs=" + enableSigns + " lights=" + enableLights);
    }

    public String modelName() {
        return modelName;
    }

    public String epName() {
        return epName;
    }

    public int inputSize() {
        return inputSize;
    }

    private static final class SessionCreated {
        final OrtSession session;
        final String epName;

        SessionCreated(OrtSession session, String epName) {
            this.session = session;
            this.epName = epName;
        }
    }

    private SessionCreated createSession(String modelPath, String prefer) throws Exception {
        // On OnePlus 7T YOLOv8n: CPU ~19 ms ORT vs NNAPI ~90 ms — prefer CPU for traffic.
        if ("nnapi".equals(prefer)) {
            OrtSession s = tryNnapi(modelPath, true);
            if (s != null) {
                return new SessionCreated(s, "nnapi_fp16");
            }
            s = tryNnapi(modelPath, false);
            if (s != null) {
                return new SessionCreated(s, "nnapi");
            }
            Log.w(TAG, "NNAPI failed — CPU");
            return new SessionCreated(createCpu(modelPath, 4), "cpu");
        }
        if ("xnnpack".equals(prefer)) {
            OrtSession s = tryXnnpack(modelPath);
            if (s != null) {
                return new SessionCreated(s, "xnnpack");
            }
            Log.w(TAG, "XNNPACK failed — CPU");
            return new SessionCreated(createCpu(modelPath, 4), "cpu");
        }
        // auto | cpu
        if ("auto".equals(prefer)) {
            OrtSession s = tryXnnpack(modelPath);
            if (s != null) {
                return new SessionCreated(s, "xnnpack");
            }
        }
        return new SessionCreated(createCpu(modelPath, 4), "cpu");
    }

    private OrtSession tryNnapi(String modelPath, boolean fp16) {
        OrtSession.SessionOptions opts = null;
        try {
            opts = newSessionOptions(2);
            if (fp16) {
                opts.addNnapi(java.util.EnumSet.of(NNAPIFlags.USE_FP16));
            } else {
                opts.addNnapi();
            }
            OrtSession s = env.createSession(modelPath, opts);
            Log.i(TAG, "NNAPI EP enabled fp16=" + fp16);
            return s;
        } catch (Throwable t) {
            Log.w(TAG, "NNAPI fp16=" + fp16 + " failed", t);
            closeQuiet(opts);
            return null;
        }
    }

    private OrtSession tryXnnpack(String modelPath) {
        OrtSession.SessionOptions opts = null;
        try {
            opts = newSessionOptions(4);
            Map<String, String> xnn = new HashMap<>();
            xnn.put("intra_op_num_threads", "4");
            opts.addXnnpack(xnn);
            OrtSession s = env.createSession(modelPath, opts);
            Log.i(TAG, "XNNPACK EP enabled");
            return s;
        } catch (Throwable t) {
            Log.w(TAG, "XNNPACK failed", t);
            closeQuiet(opts);
            return null;
        }
    }

    private OrtSession createCpu(String modelPath, int threads) throws Exception {
        OrtSession.SessionOptions opts = newSessionOptions(threads);
        Log.i(TAG, "CPU EP threads=" + threads);
        return env.createSession(modelPath, opts);
    }

    private static OrtSession.SessionOptions newSessionOptions(int intraOpThreads) throws Exception {
        OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
        opts.setIntraOpNumThreads(Math.max(1, intraOpThreads));
        opts.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
        try {
            opts.setSessionLogLevel(OrtLoggingLevel.ORT_LOGGING_LEVEL_WARNING);
        } catch (Throwable ignored) {
        }
        return opts;
    }

    private static void closeQuiet(OrtSession.SessionOptions opts) {
        if (opts == null) {
            return;
        }
        try {
            opts.close();
        } catch (Throwable ignored) {
        }
    }

    private static int readInputSize(OrtSession session, String inputName) throws Exception {
        try {
            NodeInfo ni = session.getInputInfo().get(inputName);
            if (ni != null) {
                ValueInfo vi = ni.getInfo();
                if (vi instanceof TensorInfo) {
                    long[] shape = ((TensorInfo) vi).getShape();
                    // NCHW [1,3,H,W] or NHWC [1,H,W,3]
                    if (shape != null && shape.length == 4) {
                        if (shape[2] > 0 && shape[2] == shape[3] && shape[2] <= 1280) {
                            return (int) shape[2];
                        }
                        if (shape[1] > 0 && shape[1] == shape[2] && shape[1] <= 1280) {
                            return (int) shape[1];
                        }
                    }
                }
            }
        } catch (Throwable t) {
            Log.w(TAG, "could not read input size, default " + INPUT, t);
        }
        return INPUT;
    }

    public Result run(YuvFrame yuv) throws Exception {
        long t0 = System.currentTimeMillis();
        Bitmap bmp = yuvToBitmap(yuv);
        try {
            return runBitmap(bmp);
        } finally {
            bmp.recycle();
            // keep inferMs accurate
        }
    }

    public Result runBitmap(Bitmap src) throws Exception {
        final int sz = inputSize;
        long tAll = System.nanoTime();
        int srcW = src.getWidth();
        int srcH = src.getHeight();
        float scale = Math.min(sz / (float) srcW, sz / (float) srcH);
        int nw = Math.round(srcW * scale);
        int nh = Math.round(srcH * scale);
        int padX = (sz - nw) / 2;
        int padY = (sz - nh) / 2;

        Bitmap scaled = Bitmap.createScaledBitmap(src, nw, nh, true);
        Bitmap letter = Bitmap.createBitmap(sz, sz, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(letter);
        c.drawColor(Color.rgb(114, 114, 114));
        c.drawBitmap(scaled, padX, padY, null);
        if (scaled != src) {
            scaled.recycle();
        }

        letter.getPixels(pixels, 0, sz, 0, 0, sz, sz);
        letter.recycle();
        final int n = sz * sz;
        for (int i = 0; i < n; i++) {
            int p = pixels[i];
            inputBuf[i] = ((p >> 16) & 0xff) / 255.f;
            inputBuf[n + i] = ((p >> 8) & 0xff) / 255.f;
            inputBuf[2 * n + i] = (p & 0xff) / 255.f;
        }
        int prepMs = (int) ((System.nanoTime() - tAll) / 1_000_000L);

        try (OnnxTensor tensor = OnnxTensor.createTensor(env, FloatBuffer.wrap(inputBuf),
                new long[]{1, 3, sz, sz})) {
            Map<String, OnnxTensor> feeds = new HashMap<>();
            feeds.put(inputName, tensor);
            long tOrt = System.nanoTime();
            try (OrtSession.Result out = session.run(feeds)) {
                int ortMs = (int) ((System.nanoTime() - tOrt) / 1_000_000L);
                OnnxTensor ot = (OnnxTensor) out.get(0);
                long tDec = System.nanoTime();
                List<Det> dets = decodeTensor(ot, scale, padX, padY, srcW, srcH, src);
                int decodeMs = (int) ((System.nanoTime() - tDec) / 1_000_000L);
                long tOcr = System.nanoTime();
                if (enableSigns) {
                    applySpeedOcr(dets, src, srcW, srcH);
                }
                int ocrMs = (int) ((System.nanoTime() - tOcr) / 1_000_000L);
                int totalMs = (int) ((System.nanoTime() - tAll) / 1_000_000L);
                return new Result(dets, totalMs, prepMs, ortMs, decodeMs, ocrMs, modelName, epName, inputSize);
            }
        }
    }

    /** OCR digit on crops that look like speed-limit but have no km/h in the label. */
    private static void applySpeedOcr(List<Det> dets, Bitmap src, int srcW, int srcH) {
        if (dets == null || dets.isEmpty()) {
            return;
        }
        for (Det d : dets) {
            if (d.speedLimitKmh > 0 || !needsSpeedOcr(d.label)) {
                continue;
            }
            int x1 = (int) (d.x1 * srcW);
            int y1 = (int) (d.y1 * srcH);
            int x2 = (int) (d.x2 * srcW);
            int y2 = (int) (d.y2 * srcH);
            int v = SpeedLimitOcr.readKmh(src, x1, y1, x2, y2);
            if (v > 0) {
                d.speedLimitKmh = v;
                d.speedFromOcr = true;
            }
        }
    }

    static boolean needsSpeedOcr(String label) {
        if (label == null) {
            return false;
        }
        String l = label.toLowerCase(Locale.ROOT);
        if (parseSpeedLimit(label) > 0) {
            return false;
        }
        return l.contains("ограничен") || l.contains("скорост")
                || l.contains("3_24") || l.equals("3_24")
                || (l.contains("speed") && l.contains("limit"))
                || l.contains("максимальн");
    }

    private List<Det> decodeTensor(OnnxTensor ot, float scale, int padX, int padY,
                                   int srcW, int srcH, Bitmap colorSrc) throws Exception {
        long[] shape = ot.getInfo().getShape();
        FloatBuffer fb = ot.getFloatBuffer();
        if (fb != null && shape != null && shape.length == 3) {
            // [1, C, N] or [1, N, C]
            int d1 = (int) shape[1];
            int d2 = (int) shape[2];
            boolean channelsFirst = d1 < d2 && d1 <= 512;
            int channels = channelsFirst ? d1 : d2;
            int anchors = channelsFirst ? d2 : d1;
            return decodeFlat(fb, channels, anchors, channelsFirst, scale, padX, padY, srcW, srcH, colorSrc);
        }
        // Fallback: materialize Java arrays (slower).
        float[][][] raw = (float[][][]) ot.getValue();
        return decode(raw[0], scale, padX, padY, srcW, srcH, colorSrc);
    }

    private List<Det> decodeFlat(FloatBuffer fb, int channels, int anchors, boolean channelsFirst,
                                 float scale, int padX, int padY, int srcW, int srcH, Bitmap colorSrc) {
        int nc = channels - 4;
        if (nc <= 0) {
            return Collections.emptyList();
        }
        List<Det> cand = new ArrayList<>();
        for (int i = 0; i < anchors; i++) {
            float best = 0;
            int cls = -1;
            for (int k = 0; k < nc; k++) {
                float s = at(fb, channelsFirst, channels, anchors, 4 + k, i);
                if (s > best) {
                    best = s;
                    cls = k;
                }
            }
            if (best < CONF_THRES || cls < 0) {
                continue;
            }
            String label = cls < labels.length ? labels[cls] : ("cls_" + cls);
            int speed = parseSpeedLimit(label);
            if (!keepDet(label, speed)) {
                continue;
            }
            boolean tfl = isTrafficLight(label);
            float cx = at(fb, channelsFirst, channels, anchors, 0, i);
            float cy = at(fb, channelsFirst, channels, anchors, 1, i);
            float w = at(fb, channelsFirst, channels, anchors, 2, i);
            float h = at(fb, channelsFirst, channels, anchors, 3, i);
            float x1 = (cx - w * 0.5f - padX) / scale;
            float y1 = (cy - h * 0.5f - padY) / scale;
            float x2 = (cx + w * 0.5f - padX) / scale;
            float y2 = (cy + h * 0.5f - padY) / scale;
            x1 = clamp(x1, 0, srcW - 1);
            y1 = clamp(y1, 0, srcH - 1);
            x2 = clamp(x2, 0, srcW - 1);
            y2 = clamp(y2, 0, srcH - 1);
            if (x2 - x1 < 2 || y2 - y1 < 2) {
                continue;
            }
            Det d = new Det();
            d.label = label;
            d.conf = best;
            d.x1 = x1 / srcW;
            d.y1 = y1 / srcH;
            d.x2 = x2 / srcW;
            d.y2 = y2 / srcH;
            d.speedLimitKmh = speed;
            d.tflColor = (tfl && enableLights)
                    ? classifyLightColor(colorSrc, (int) x1, (int) y1, (int) x2, (int) y2)
                    : 0;
            cand.add(d);
        }
        return nms(cand);
    }

    private static float at(FloatBuffer fb, boolean channelsFirst, int channels, int anchors,
                            int c, int i) {
        int idx = channelsFirst ? c * anchors + i : i * channels + c;
        return fb.get(idx);
    }

    private List<Det> decode(float[][] pred, float scale, int padX, int padY, int srcW, int srcH,
                             Bitmap colorSrc) {
        // pred is either [4+nc][N] (ultralytics) or [N][4+nc]
        boolean channelsFirst = pred.length < pred[0].length && pred.length <= 512;
        int rows;
        int cols;
        float[][] data;
        if (channelsFirst) {
            // [C, N] → treat as C rows
            cols = pred.length;
            rows = pred[0].length;
            data = pred;
        } else {
            // [N, C]
            rows = pred.length;
            cols = pred[0].length;
            data = new float[cols][rows];
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    data[j][i] = pred[i][j];
                }
            }
        }
        int nc = cols - 4;
        if (nc <= 0) {
            return Collections.emptyList();
        }
        List<Det> cand = new ArrayList<>();
        for (int i = 0; i < rows; i++) {
            float best = 0;
            int cls = -1;
            for (int k = 0; k < nc; k++) {
                float s = data[4 + k][i];
                if (s > best) {
                    best = s;
                    cls = k;
                }
            }
            if (best < CONF_THRES || cls < 0) {
                continue;
            }
            float cx = data[0][i];
            float cy = data[1][i];
            float w = data[2][i];
            float h = data[3][i];
            // undo letterbox
            float x1 = (cx - w * 0.5f - padX) / scale;
            float y1 = (cy - h * 0.5f - padY) / scale;
            float x2 = (cx + w * 0.5f - padX) / scale;
            float y2 = (cy + h * 0.5f - padY) / scale;
            x1 = clamp(x1, 0, srcW - 1);
            y1 = clamp(y1, 0, srcH - 1);
            x2 = clamp(x2, 0, srcW - 1);
            y2 = clamp(y2, 0, srcH - 1);
            if (x2 - x1 < 2 || y2 - y1 < 2) {
                continue;
            }
            Det d = new Det();
            d.label = cls < labels.length ? labels[cls] : ("cls_" + cls);
            d.conf = best;
            d.x1 = x1 / srcW;
            d.y1 = y1 / srcH;
            d.x2 = x2 / srcW;
            d.y2 = y2 / srcH;
            d.speedLimitKmh = parseSpeedLimit(d.label);
            d.tflColor = 0;
            if (isTrafficLight(d.label)) {
                d.tflColor = enableLights
                        ? classifyLightColor(colorSrc, (int) x1, (int) y1, (int) x2, (int) y2)
                        : 0;
            }
            if (keepDet(d.label, d.speedLimitKmh)) {
                cand.add(d);
            }
        }
        return nms(cand);
    }

    /** Classes we surface on HUD, gated by feature flags. */
    private boolean keepDet(String label, int speedKmh) {
        if (isTrafficLight(label)) {
            return enableLights;
        }
        if (speedKmh > 0 || isSignClass(label) || isPersonOrVru(label)) {
            return enableSigns;
        }
        return false;
    }

    private static boolean isTrafficLight(String label) {
        String l = label.toLowerCase(Locale.US);
        return l.contains("traffic") && l.contains("light")
                || l.equals("tfl") || l.startsWith("traffic_light")
                || l.contains("светофор");
    }

    private static boolean isPersonOrVru(String label) {
        String l = label.toLowerCase(Locale.US);
        // Crosswalk is a sign, not a person — handled in isSignClass.
        if (l.contains("переход")) {
            return false;
        }
        return l.equals("person") || l.contains("pedestrian") || l.equals("пешеход")
                || l.equals("bicycle") || l.equals("motorcycle") || l.contains("велосипед");
    }

    private static boolean isSignClass(String label) {
        String l = label.toLowerCase(Locale.ROOT);
        return l.contains("stop") && l.contains("sign")
                || l.equals("stop")
                || l.startsWith("3_") || l.startsWith("2_") || l.startsWith("1_")
                || l.startsWith("4_") || l.startsWith("5_") || l.contains("speed")
                || l.contains("limit") || l.contains("знак") || l.contains("sign")
                // RU nhassl3 / RTSD-style display names
                || l.contains("ограничение") || l.contains("скорост")
                || l.contains("переход") || l.contains("уступи") || l.contains("главн")
                || l.contains("запрещ") || l.contains("дети") || l.contains("жилая")
                || l.contains("заправк") || l.contains("работ") || l.contains("неровн")
                || l.contains("грузовик") || l.contains("автобус") || l.contains("вело");
    }

    static int parseSpeedLimit(String label) {
        if (label == null) {
            return 0;
        }
        Matcher m = SPEED_PAT.matcher(label);
        if (!m.find()) {
            return 0;
        }
        String g = m.group(1) != null ? m.group(1) : m.group(2);
        if (g == null) {
            return 0;
        }
        try {
            int v = Integer.parseInt(g);
            if (v >= 5 && v <= 150 && v % 5 == 0) {
                return v;
            }
        } catch (NumberFormatException ignored) {
        }
        return 0;
    }

    /** HSV dominant-channel heuristic on crop. */
    static int classifyLightColor(Bitmap bmp, int x1, int y1, int x2, int y2) {
        int w = Math.max(1, x2 - x1);
        int h = Math.max(1, y2 - y1);
        int stepX = Math.max(1, w / 16);
        int stepY = Math.max(1, h / 24);
        int red = 0, yellow = 0, green = 0, n = 0;
        float[] hsv = new float[3];
        for (int y = y1; y < y2; y += stepY) {
            for (int x = x1; x < x2; x += stepX) {
                int p = bmp.getPixel(x, y);
                Color.colorToHSV(p, hsv);
                float s = hsv[1];
                float v = hsv[2];
                if (v < 0.35f || s < 0.25f) {
                    continue;
                }
                float hue = hsv[0];
                n++;
                if (hue < 20 || hue > 340) {
                    red++;
                } else if (hue >= 20 && hue < 55) {
                    yellow++;
                } else if (hue >= 70 && hue < 170) {
                    green++;
                }
            }
        }
        if (n < 3) {
            return 4; // off / unknown dim
        }
        if (red >= yellow && red >= green) {
            return 1;
        }
        if (yellow >= green) {
            return 2;
        }
        return 3;
    }

    private static List<Det> nms(List<Det> dets) {
        Collections.sort(dets, (a, b) -> Float.compare(b.conf, a.conf));
        boolean[] gone = new boolean[dets.size()];
        List<Det> out = new ArrayList<>();
        for (int i = 0; i < dets.size(); i++) {
            if (gone[i]) {
                continue;
            }
            Det a = dets.get(i);
            out.add(a);
            for (int j = i + 1; j < dets.size(); j++) {
                if (gone[j]) {
                    continue;
                }
                Det b = dets.get(j);
                if (!a.label.equals(b.label) && !(isTrafficLight(a.label) && isTrafficLight(b.label))) {
                    continue;
                }
                if (iou(a, b) > IOU_THRES) {
                    gone[j] = true;
                }
            }
        }
        return out;
    }

    private static float iou(Det a, Det b) {
        float xx1 = Math.max(a.x1, b.x1);
        float yy1 = Math.max(a.y1, b.y1);
        float xx2 = Math.min(a.x2, b.x2);
        float yy2 = Math.min(a.y2, b.y2);
        float w = Math.max(0, xx2 - xx1);
        float h = Math.max(0, yy2 - yy1);
        float inter = w * h;
        float uni = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
        return uni > 0 ? inter / uni : 0;
    }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private Bitmap yuvToBitmap(YuvFrame yuv) {
        int w = yuv.width;
        int h = yuv.height;
        int[] argb = new int[w * h];
        for (int j = 0; j < h; j++) {
            int uvRow = (j >> 1) * (w >> 1);
            for (int i = 0; i < w; i++) {
                int Y = yuv.y[j * w + i] & 0xff;
                int U = yuv.u[uvRow + (i >> 1)] & 0xff;
                int V = yuv.v[uvRow + (i >> 1)] & 0xff;
                int c = Y - 16;
                int d = U - 128;
                int e = V - 128;
                int r = clamp255((298 * c + 409 * e + 128) >> 8);
                int g = clamp255((298 * c - 100 * d - 208 * e + 128) >> 8);
                int b = clamp255((298 * c + 516 * d + 128) >> 8);
                argb[j * w + i] = 0xff000000 | (r << 16) | (g << 8) | b;
            }
        }
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        bmp.setPixels(argb, 0, w, 0, 0, w, h);
        return bmp;
    }

    private static int clamp255(int v) {
        return v < 0 ? 0 : (v > 255 ? 255 : v);
    }

    private String[] loadLabels(Context context) {
        // Prefer labels next to sdcard model, then assets, else COCO-80.
        String[] fromSd = tryReadFileLines(new File("/sdcard/adas_models/traffic_labels.txt"));
        if (fromSd != null && fromSd.length > 0) {
            return fromSd;
        }
        String[] fromAsset = tryReadLines(context, "traffic_labels.txt");
        if (fromAsset != null && fromAsset.length > 0) {
            return fromAsset;
        }
        return COCO80;
    }

    private static String[] tryReadFileLines(File file) {
        if (file == null || !file.isFile()) {
            return null;
        }
        try (BufferedReader br = new BufferedReader(new java.io.FileReader(file))) {
            List<String> lines = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.isEmpty() && !line.startsWith("#")) {
                    lines.add(line);
                }
            }
            return lines.toArray(new String[0]);
        } catch (Exception e) {
            return null;
        }
    }

    private static String[] tryReadLines(Context context, String asset) {
        try (InputStream in = context.getAssets().open(asset);
             BufferedReader br = new BufferedReader(new InputStreamReader(in))) {
            List<String> lines = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.isEmpty() && !line.startsWith("#")) {
                    lines.add(line);
                }
            }
            return lines.toArray(new String[0]);
        } catch (Exception e) {
            return null;
        }
    }

    static File resolveModelFile(Context context, String assetName) throws Exception {
        File sd = new File("/sdcard/adas_models/" + assetName);
        if (sd.isFile() && sd.length() > 1000) {
            return sd;
        }
        File cached = new File(context.getFilesDir(), assetName);
        long assetLen = -1;
        try (InputStream in = context.getAssets().open(assetName)) {
            assetLen = in.available();
        } catch (Exception ignored) {
        }
        boolean needCopy = !cached.isFile() || cached.length() < 1000
                || (assetLen > 1000 && Math.abs(cached.length() - assetLen) > 1024);
        if (!needCopy) {
            return cached;
        }
        try (InputStream in = context.getAssets().open(assetName);
             FileOutputStream out = new FileOutputStream(cached)) {
            byte[] buf = new byte[1 << 16];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
            }
        }
        return cached;
    }

    public void close() {
        try {
            session.close();
        } catch (Exception ignored) {
        }
    }

    /** COCO-80 — index 9 is traffic light. */
    private static final String[] COCO80 = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
    };
}
