package ai.flow.adas;

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
