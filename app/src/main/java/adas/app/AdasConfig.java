package adas.app;

import android.content.Context;
import android.util.Log;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

public final class AdasConfig {
    private static final String TAG = "AdasConfig";
    public static final String ASSET = "config.json";
    private static final String DEFAULT_MODEL = "supercombo.onnx";
    private static final String DEFAULT_MAP = "Moscow.osm.admap";

    private AdasConfig() {}

    private static JSONObject root(Context context) throws Exception {
        File file = RuntimeParams.configFile(context);
        if (file.exists() && file.length() > 0) {
            try {
                return parseStream(new FileInputStream(file));
            } catch (Exception e) {
                Log.w(TAG, "Corrupt override — falling back to asset", e);
            }
        }
        return parseStream(context.getAssets().open(ASSET));
    }

    private static JSONObject parseStream(InputStream in) throws Exception {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                sb.append(line).append('\n');
            }
            return new JSONObject(sb.toString());
        }
    }

    public static String supercomboAsset(Context context) {
        try {
            String name = root(context).optString("supercombo_asset", DEFAULT_MODEL);
            if (name == null || name.isEmpty()) {
                name = DEFAULT_MODEL;
            }
            Log.i(TAG, "supercombo_asset=" + name);
            return name;
        } catch (Exception e) {
            Log.w(TAG, "Failed to read " + ASSET + ", using " + DEFAULT_MODEL, e);
            return DEFAULT_MODEL;
        }
    }

    public static String trafficYoloAsset(Context context) {
        try {
            String name = root(context).optString("traffic_yolo_asset", "traffic_yolo.onnx");
            if (name == null || name.isEmpty()) {
                name = "traffic_yolo.onnx";
            }
            return name;
        } catch (Exception e) {
            return "traffic_yolo.onnx";
        }
    }

    public static boolean visionSupercomboEnabled(Context context) {
        try {
            JSONObject nodes = root(context).optJSONObject("nodes");
            if (nodes == null) {
                return true;
            }
            return nodes.optBoolean("vision_supercombo", true);
        } catch (Exception e) {
            return true;
        }
    }

    public static boolean visionTrafficEnabled(Context context) {
        try {
            JSONObject nodes = root(context).optJSONObject("nodes");
            if (nodes == null) {
                return true;
            }
            if (!nodes.optBoolean("vision_traffic", true)) {
                return false;
            }
            return visionTrafficSignsEnabled(context) || visionTrafficLightsEnabled(context);
        } catch (Exception e) {
            return true;
        }
    }

    public static boolean visionTrafficSignsEnabled(Context context) {
        return nodeBool(context, "vision_traffic_signs", true);
    }

    public static boolean visionTrafficLightsEnabled(Context context) {
        return nodeBool(context, "vision_traffic_lights", true);
    }

    public static boolean chessboardCapture(Context context) {
        try {
            org.json.JSONObject calib = root(context).optJSONObject("calibration");
            org.json.JSONObject cam = calib == null ? null : calib.optJSONObject("camera");
            boolean on = cam != null && cam.optBoolean("chessboard_capture", false);
            if (on) {
                Log.w(TAG, "camera.chessboard_capture=true: autofocus enabled (calibration only)");
            }
            return on;
        } catch (Exception e) {
            Log.w(TAG, "chessboardCapture read failed", e);
            return false;
        }
    }

    public static boolean phoneStatsEnabled(Context context) {
        return nodeBool(context, "phone_stats", true);
    }

    /** {@code nodes.map_data} — road curvature ahead from the OSM map. Off by default; see
     *  {@code docs/MAP_CURVATURE.md}. Gates unpacking the 4.9 MB map asset as well as the C++ service. */
    public static boolean mapDataEnabled(Context context) {
        return nodeBool(context, "map_data", false);
    }

    /**
     * Asset name of the road map, from {@code map.path}.
     *
     * <p>The same key the C++ side reads, so the two cannot drift: Java unpacks whatever it names and hands
     * back the absolute path, and {@code nativeStart} overrides the configured value with it.
     */
    public static String mapAsset(Context context) {
        try {
            JSONObject map = root(context).optJSONObject("map");
            String name = map == null ? null : map.optString("path", DEFAULT_MAP);
            if (name == null || name.isEmpty()) {
                name = DEFAULT_MAP;
            }
            return name;
        } catch (Exception e) {
            Log.w(TAG, "Failed to read map.path, using " + DEFAULT_MAP, e);
            return DEFAULT_MAP;
        }
    }

    /**
     * Which supercombo to run ({@code vision.model_runner}): {@code onnx} or {@code thneed}.
     *
     * <p>{@code thneed} is the flowpilot 0.9.x model on the GPU — 15.9 ms against 44.7 for ours, which
     * is what lets the pipeline hold 30 Hz ({@code docs/VISION_RATE.md}). Anything unrecognised falls
     * back to {@code onnx}.
     */
    public static String modelRunner(Context context) {
        try {
            JSONObject vision = root(context).optJSONObject("vision");
            String v = vision == null ? null : vision.optString("model_runner", "onnx");
            return "thneed".equalsIgnoreCase(v) ? "thneed" : "onnx";
        } catch (Exception e) {
            return "onnx";
        }
    }


    /**
     * Let NNAPI compute supercombo in half precision ({@code vision.nnapi_fp16}).
     *
     * <p>Worth roughly 15-20 ms of the 45.6 ms inference on this SoC, which would take the vision
     * loop from 13.2 Hz to about 20 Hz — the single largest remaining pipeline lever, since
     * everything else in the cycle adds up to 26 ms with no slack.
     *
     * <p>Checked offline before wiring, on 200 frames of run 2026_08_06_00_36_42 where both lane
     * lines are visible ({@code bag_fp16_ab.py}). Converting the whole model to fp16 — a stricter
     * test than NNAPI, which keeps a float32 graph and is merely allowed to relax precision per node
     * — moved the lane centre by 0.027 m and the plan offset by 0.037 m in aggregate, while line
     * probabilities went up (right line 0.37 to 0.82) and the sigma tail shrank (p90 0.69 to 0.33).
     * So not a degradation. What it is not free of: per-frame disagreement of 0.05 m median and
     * 0.20 m p95 on the lane centre.
     *
     * <p><b>Default off on purpose.</b> The pending sigma-threshold experiment
     * ({@code lane_std_bad_m} 1.5 to 2.0) also moves the lateral chain, and two changes in one drive
     * cannot be separated. Enable it as its own single-variable run.
     */
    public static boolean nnapiFp16(Context context) {
        try {
            JSONObject vision = root(context).optJSONObject("vision");
            return vision != null && vision.optBoolean("nnapi_fp16", false);
        } catch (Exception e) {
            return false;
        }
    }

    private static boolean nodeBool(Context context, String key, boolean def) {
        try {
            JSONObject nodes = root(context).optJSONObject("nodes");
            if (nodes == null) {
                return def;
            }
            return nodes.optBoolean(key, def);
        } catch (Exception e) {
            return def;
        }
    }
}
