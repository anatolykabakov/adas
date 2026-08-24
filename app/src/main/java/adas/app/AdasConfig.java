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

    /** \brief {@code nodes.vision_traffic} — the traffic-light detector. */
    public static boolean visionTrafficEnabled(Context context) {
        return nodeBool(context, "vision_traffic", false);
    }

    /** Known cars: the name for `vehicle.name` and the DBC that parses them. */
    public static final String[][] CARS = {
        {"vw_golf_7_mqb", "VW Golf 7 (MQB)", "vw_mqb_2010.dbc"},
    };

    /** Which car is selected — `vehicle.name` from the config. */
    public static String carName(Context context) {
        try {
            JSONObject veh = root(context).optJSONObject("vehicle");
            String name = veh != null ? veh.optString("name", "") : "";
            return name != null ? name.trim() : "";
        } catch (Exception e) {
            Log.w(TAG, "carName read failed", e);
            return "";
        }
    }

    /** DBC asset for the selected car; an empty string if the car is unknown. */
    public static String dbcAssetFor(String carName) {
        for (String[] car : CARS) {
            if (car[0].equals(carName)) {
                return car[2];
            }
        }
        return "";
    }

    /** The phone model `intrinsics_prior` was taken on, or an empty string. */
    public static String intrinsicsPriorDevice(Context context) {
        try {
            org.json.JSONObject calib = root(context).optJSONObject("calibration");
            org.json.JSONObject cam = calib == null ? null : calib.optJSONObject("camera");
            org.json.JSONObject intr = cam == null ? null : cam.optJSONObject("intrinsics_prior");
            return intr == null ? "" : intr.optString("device", "");
        } catch (Exception e) {
            Log.w(TAG, "intrinsicsPriorDevice read failed", e);
            return "";
        }
    }

    /**
     * Reference output signature on zero inputs for the named runner, or null.
     * @param runner "onnx" or "thneed" — their precision differs, so their signatures differ
     * @return {@code {mean, std}}, or null if nothing is set in the config
     */
    public static float[] zeroInputSignature(Context context, String runner) {
        try {
            org.json.JSONObject vision = root(context).optJSONObject("vision");
            org.json.JSONObject sig = vision == null ? null : vision.optJSONObject("zero_input");
            org.json.JSONObject one = sig == null ? null : sig.optJSONObject(runner);
            if (one == null || !one.has("mean") || !one.has("std")) {
                return null;
            }
            return new float[]{(float) one.optDouble("mean"), (float) one.optDouble("std")};
        } catch (Exception e) {
            Log.w(TAG, "zeroInputSignature read failed", e);
            return null;
        }
    }

    public static boolean phoneStatsEnabled(Context context) {
        return nodeBool(context, "phone_stats", true);
    }



    /** Which supercombo to run ({@code vision.model_runner}): {@code onnx} or {@code thneed}. */
    public static String modelRunner(Context context) {
        try {
            JSONObject vision = root(context).optJSONObject("vision");
            String v = vision == null ? null : vision.optString("model_runner", "onnx");
            return "thneed".equalsIgnoreCase(v) ? "thneed" : "onnx";
        } catch (Exception e) {
            return "onnx";
        }
    }


    /** Let NNAPI compute supercombo in half precision ({@code vision.nnapi_fp16}). */
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
