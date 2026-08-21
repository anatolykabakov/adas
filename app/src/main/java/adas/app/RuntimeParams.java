package adas.app;

import android.content.Context;
import android.util.Log;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;

public final class RuntimeParams {
    private static final String TAG = "RuntimeParams";

    public float rollDeg = 0f;
    public float pitchDeg = 0f;
    public float yawDeg = 0f;
    public float heightM = 1.1f;
    public float camX = 1.50f;
    public float camY = 0f;

    /** Take the focal length from the camera itself and keep {@code intrinsics_prior} as a fallback. */

    public boolean intrinsicsFromDevice = true;

    /** The phone model {@code intrinsics_prior} was taken on, from the {@code device} field inside it. */
    public String intrinsicsPriorDevice = "";


    public float fx = 930f;
    public float fy = 930f;
    public float cx = 640f;
    public float cy = 360f;
    public int calibWidth = 1280;
    public int calibHeight = 720;

    public float ppKdd = 0.4f;
    public float ppLdMin = 3.0f;
    public float ppLdMax = 20.0f;
    public float wheelbaseM = 2.636f;
    public float ppShift = 1.4f;
    public float steerRatio = 15.7f;
    public String laneKeepController = "fp";
    public float laneBlendScale = 0.6f;
    public boolean recordCameraImages = true;

    public String supercomboAsset = "supercombo.onnx";

    /**
     * Vision model: {@code onnx} or {@code thneed}. Same key as
     * {@link AdasConfig#modelRunner} reads, so the settings switch and the native side cannot disagree.
     */
    public String modelRunner = "onnx";

    /** Camera rate, and with it the spacing of the frame pair the model reads: 20 or 30 fps. */
    public int cameraFps = adas.app.sensors.CameraHandler.FPS_MODEL;

    public static String normalizeModelRunner(String v) {
        return "thneed".equalsIgnoreCase(v) ? "thneed" : "onnx";
    }

    /** Sign recognition by a second network, switched from the params panel. */

    /** Only the two offered rates; anything else falls back to the model's own. */
    public static int normalizeCameraFps(int fps) {
        return fps == adas.app.sensors.CameraHandler.FPS_FAST
                ? adas.app.sensors.CameraHandler.FPS_FAST
                : adas.app.sensors.CameraHandler.FPS_MODEL;
    }

    public static String normalizeController(String ctrl) {
        if (ctrl == null) {
            return "pp";
        }
        if ("mpc".equalsIgnoreCase(ctrl)) {
            return "mpc";
        }
        if ("fp".equalsIgnoreCase(ctrl) || "flowpilot".equalsIgnoreCase(ctrl)) {
            return "fp";
        }
        return "pp";
    }

    public static File configFile(Context context) {
        return new File(context.getFilesDir(), AdasConfig.ASSET);
    }

    public static RuntimeParams load(Context context) {
        RuntimeParams p = new RuntimeParams();
        JSONObject root = readJson(context);
        if (root == null) {
            return p;
        }
        try {
            JSONObject vehicle = root.optJSONObject("vehicle");
            if (vehicle != null) {
                p.wheelbaseM = (float) vehicle.optDouble("wheelbase_m", p.wheelbaseM);
                p.steerRatio = (float) vehicle.optDouble("steer_ratio", p.steerRatio);
                p.ppKdd = (float) vehicle.optDouble("pp_k_dd", p.ppKdd);
                p.ppLdMin = (float) vehicle.optDouble("pp_ld_min", p.ppLdMin);
                p.ppLdMax = (float) vehicle.optDouble("pp_ld_max", p.ppLdMax);
                p.ppShift = (float) vehicle.optDouble("pp_shift", p.ppShift);
                p.laneBlendScale = (float) vehicle.optDouble("path_lane_blend_scale", p.laneBlendScale);
                String ctrl = vehicle.optString("lane_keep_controller", p.laneKeepController);
                p.laneKeepController = normalizeController(ctrl);
            }
            JSONObject logging = root.optJSONObject("logging");
            if (logging != null) {
                p.recordCameraImages = logging.optBoolean("record_camera_images", p.recordCameraImages);
            }
            JSONObject vision = root.optJSONObject("vision");
            if (vision != null) {
                p.modelRunner = normalizeModelRunner(vision.optString("model_runner", p.modelRunner));
                p.cameraFps = normalizeCameraFps(vision.optInt("camera_fps", p.cameraFps));
            }
            JSONObject cam = root.optJSONObject("calibration");
            if (cam != null) {
                cam = cam.optJSONObject("camera");
            }
            if (cam != null) {
                JSONObject pos = cam.optJSONObject("position_m");
                if (pos != null) {
                    p.camX = (float) pos.optDouble("x_forward", p.camX);
                    p.camY = (float) pos.optDouble("y_left", p.camY);
                    p.heightM = (float) pos.optDouble("z_up", p.heightM);
                }
                JSONObject rpy = cam.optJSONObject("rpy_deg");
                if (rpy != null) {
                    p.rollDeg = (float) rpy.optDouble("roll", p.rollDeg);
                    p.pitchDeg = (float) rpy.optDouble("pitch", p.pitchDeg);
                    p.yawDeg = (float) rpy.optDouble("yaw", p.yawDeg);
                }
                p.intrinsicsFromDevice = cam.optBoolean("intrinsics_from_device", p.intrinsicsFromDevice);
                JSONObject intr = cam.optJSONObject("intrinsics_prior");
                if (intr != null) {
                    p.intrinsicsPriorDevice = intr.optString("device", p.intrinsicsPriorDevice);
                    p.fx = (float) intr.optDouble("fx", p.fx);
                    p.fy = (float) intr.optDouble("fy", p.fy);
                    p.cx = (float) intr.optDouble("cx", p.cx);
                    p.cy = (float) intr.optDouble("cy", p.cy);
                    p.calibWidth = intr.optInt("width", p.calibWidth);
                    p.calibHeight = intr.optInt("height", p.calibHeight);
                }
            }
            p.supercomboAsset = root.optString("supercombo_asset", p.supercomboAsset);
        } catch (Exception e) {
            Log.w(TAG, "parse failed", e);
        }
        return p;
    }

    /** Replace the config in filesDir atomically. */
    private static void writeConfig(Context context, JSONObject root) throws Exception {
        File out = configFile(context);
        File tmp = new File(out.getParentFile(), out.getName() + ".tmp");
        byte[] bytes = root.toString(2).getBytes(StandardCharsets.UTF_8);
        try (FileOutputStream fos = new FileOutputStream(tmp);
             FileChannel ch = fos.getChannel()) {
            fos.write(bytes);
            fos.flush();
            ch.force(true);
        }
        if (!tmp.renameTo(out)) {
            if (out.exists() && !out.delete()) {
                throw new Exception("Cannot replace " + out.getAbsolutePath());
            }
            if (!tmp.renameTo(out)) {
                throw new Exception("Atomic rename failed for " + out.getAbsolutePath());
            }
        }
        Log.i(TAG, "Saved " + out.getAbsolutePath());
    }

    /**
     * Write the selected car into `vehicle.name` without touching the rest of the config.
     * @param context Application context.
     * @param carName A name from {@link AdasConfig#CARS}.
     */
    public static void setCarName(Context context, String carName) throws Exception {
        JSONObject root = readJson(context);
        if (root == null) {
            throw new IllegalStateException("config is not readable");
        }
        JSONObject veh = root.optJSONObject("vehicle");
        if (veh == null) {
            veh = new JSONObject();
            root.put("vehicle", veh);
        }
        veh.put("name", carName);
        writeConfig(context, root);
    }

    public void save(Context context) throws Exception {
        JSONObject root = readJson(context);
        if (root == null) {
            root = new JSONObject();
        }
        JSONObject vehicle = root.optJSONObject("vehicle");
        if (vehicle == null) {
            vehicle = new JSONObject();
            root.put("vehicle", vehicle);
        }
        vehicle.put("wheelbase_m", wheelbaseM);
        vehicle.put("steer_ratio", steerRatio);
        vehicle.put("pp_k_dd", ppKdd);
        vehicle.put("pp_ld_min", ppLdMin);
        vehicle.put("pp_ld_max", ppLdMax);
        vehicle.put("pp_shift", ppShift);
        vehicle.put("path_lane_blend_scale", laneBlendScale);
        vehicle.put("lane_keep_controller", normalizeController(laneKeepController));

        JSONObject logging = root.optJSONObject("logging");
        if (logging == null) {
            logging = new JSONObject();
            root.put("logging", logging);
        }
        logging.put("record_camera_images", recordCameraImages);

        // Only our own field: the comment_* keys live alongside and must survive.
        JSONObject vision = root.optJSONObject("vision");
        if (vision == null) {
            vision = new JSONObject();
            root.put("vision", vision);
        }
        vision.put("model_runner", normalizeModelRunner(modelRunner));
        vision.put("camera_fps", normalizeCameraFps(cameraFps));

        JSONObject calib = root.optJSONObject("calibration");
        if (calib == null) {
            calib = new JSONObject();
            root.put("calibration", calib);
        }
        JSONObject cam = calib.optJSONObject("camera");
        if (cam == null) {
            cam = new JSONObject();
            calib.put("camera", cam);
        }
        JSONObject pos = cam.optJSONObject("position_m");
        if (pos == null) {
            pos = new JSONObject();
            cam.put("position_m", pos);
        }
        pos.put("x_forward", camX);
        pos.put("y_left", camY);
        pos.put("z_up", heightM);

        JSONObject rpy = cam.optJSONObject("rpy_deg");
        if (rpy == null) {
            rpy = new JSONObject();
            cam.put("rpy_deg", rpy);
        }
        rpy.put("roll", rollDeg);
        rpy.put("pitch", pitchDeg);
        rpy.put("yaw", yawDeg);

        JSONObject intr = cam.optJSONObject("intrinsics_prior");
        if (intr == null) {
            intr = new JSONObject();
            cam.put("intrinsics_prior", intr);
        }
        intr.put("fx", fx);
        intr.put("fy", fy);
        intr.put("cx", cx);
        intr.put("cy", cy);
        intr.put("width", calibWidth);
        intr.put("height", calibHeight);

        root.put("supercombo_asset", supercomboAsset);

        writeConfig(context, root);
    }

    private static JSONObject readJson(Context context) {
        File file = configFile(context);
        if (file.exists() && file.length() > 0) {
            try {
                return parseStream(new FileInputStream(file));
            } catch (Exception e) {
                Log.w(TAG, "Corrupt override " + file.getAbsolutePath() + " — falling back to asset", e);
            }
        }
        try {
            return parseStream(context.getAssets().open(AdasConfig.ASSET));
        } catch (Exception e) {
            Log.w(TAG, "readJson asset failed", e);
            return null;
        }
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
}
