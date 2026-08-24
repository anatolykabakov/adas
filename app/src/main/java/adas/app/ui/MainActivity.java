package adas.app.ui;

import adas.app.R;
import adas.app.AdasAppHandler;
import adas.app.AdasConfig;
import adas.app.Logger;
import adas.app.RuntimeParams;
import adas.app.bridge.ZMQBridgeService;
import adas.app.sensors.CameraHandler;
import adas.app.sensors.GPSHandler;
import adas.app.sensors.IMUHandler;
import adas.app.sensors.PhoneStatsHandler;
import adas.proto.CameraCalibOuter;
import adas.proto.LaneKeepOuter;
import adas.proto.LanePathOuterClass;
import adas.proto.Panda;
import adas.proto.SafetyWarnOuter;
import adas.proto.SteerOuter;
import adas.proto.TrafficVisionOuter;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.Settings;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.ImageButton;
import android.widget.RadioGroup;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;

import adas.proto.Messages.ZMQMessage;
import adas.app.vision.LaneOverlayView;
import adas.app.vision.VisionPipeline;

public class MainActivity extends AppCompatActivity {
    public static final String LOG_TAG = "MainActivity";
    private static final int REQ_PERMISSIONS = 1;
    private static final long CAN_ONLINE_TIMEOUT_MS = 500;

    private ImageButton loggingToggleButton;
    private ImageButton paramsToggleButton;
    private View canOnlineLed;
    private TextView canOnlineText;
    private ScrollView paramsPanel;
    private TextureView mImageView;
    private LaneOverlayView laneOverlay;

    private CameraHandler cameraHandler;
    private GPSHandler gpsHandler;
    private IMUHandler imuHandler;
    private PhoneStatsHandler phoneStatsHandler;
    /** Swapped from {@link #visionRebuild} when the model changes; read from the UI and ZMQ threads. */
    private volatile VisionPipeline visionPipeline;
    /** Single thread so two quick taps cannot rebuild concurrently and orphan a pipeline. */
    private final java.util.concurrent.ExecutorService visionRebuild =
            java.util.concurrent.Executors.newSingleThreadExecutor(r -> new Thread(r, "VisionRebuild"));

    /** Lens calibration runs on the pipeline's own camera stream; null while it is not running. */
    private adas.app.vision.IntrinsicsCalibrator intrinsicsCalibrator;
    private final java.util.concurrent.atomic.AtomicBoolean calibBusy =
            new java.util.concurrent.atomic.AtomicBoolean(false);
    private java.util.concurrent.ExecutorService calibDetect;
    private int calibFrameCounter = 0;
    /** Detection costs tens of milliseconds; the camera runs at 30 Hz and does not need every frame. */
    private static final int CALIB_DETECT_EVERY_N = 5;

    /** Writing the config is file I/O and this listener runs on the main thread. */
    private final java.util.concurrent.ExecutorService calibSave =
            java.util.concurrent.Executors.newSingleThreadExecutor(r -> new Thread(r, "CalibSave"));

    /** Least time between two automatic writes [ms]. The estimate keeps moving in the last decimal
     *  long after it is settled, and flash is not free. */
    private static final long CALIB_SAVE_MIN_INTERVAL_MS = 60_000L;
    /** Least change worth a write [deg]. Below this the overlay does not visibly move. */
    private static final float CALIB_SAVE_MIN_DELTA_DEG = 0.05f;

    private long lastCalibSaveMs = 0L;
    private float savedRollDeg = Float.NaN;
    private float savedPitchDeg = Float.NaN;
    private float savedYawDeg = Float.NaN;
    private adas.app.vision.TrafficVisionPipeline trafficVisionPipeline;

    private boolean cameraStarted;
    private boolean storagePromptShown;
    private boolean paramsOpen;

    private RuntimeParams params = new RuntimeParams();
    private android.widget.CheckBox recordImagesCheck;
    private RadioGroup laneKeepControllerGroup;
    private RadioGroup modelRunnerGroup;
    private RadioGroup cameraFpsGroup;
    private android.widget.Spinner carSpinner;

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final Runnable canStatusTick = new Runnable() {
        @Override
        public void run() {
            updateCanOnlineUi();
            uiHandler.postDelayed(this, 200);
        }
    };

    private final ZMQBridgeService.OutboundListener outboundListener =
            (topic, message) -> uiHandler.post(() -> onOutboundMessage(topic, message));

    @RequiresApi(api = Build.VERSION_CODES.M)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // The screen must not go dark: the activity pauses, the camera stops, and the system stops
        // seeing the road — silently, mid-drive. Found on the Xiaomi 14, where the screen timeout is
        // shorter: the pipeline ran for a minute and stopped while the app stayed in the foreground.
        getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        loggingToggleButton = findViewById(R.id.loggingToggleButton);
        paramsToggleButton = findViewById(R.id.paramsToggleButton);
        canOnlineLed = findViewById(R.id.canOnlineLed);
        canOnlineText = findViewById(R.id.canOnlineText);
        paramsPanel = findViewById(R.id.paramsPanel);
        mImageView = findViewById(R.id.textureView);
        laneOverlay = findViewById(R.id.laneOverlay);

        cameraHandler = new CameraHandler(getApplication().getApplicationContext(), mImageView);
        // The camera's focal length is known only from the first frame; once we learn it, rebuild the
        // warp at once so vision does not run on the config's number longer than necessary.
        cameraHandler.setOnIntrinsicsReady(() -> runOnUiThread(this::applyParamsToVision));
        cameraHandler.setFailureListener(reason -> uiHandler.post(() -> {
            stopCamera();
            Toast.makeText(MainActivity.this, "Camera failed: " + reason, Toast.LENGTH_LONG).show();
        }));
        gpsHandler = new GPSHandler(this);
        imuHandler = new IMUHandler(this);
        if (AdasConfig.phoneStatsEnabled(this)) {
            phoneStatsHandler = new PhoneStatsHandler(this);
        } else {
            Log.i(LOG_TAG, "phone_stats disabled in config");
        }
        startService(new Intent(getApplicationContext(), AdasAppHandler.class));

        params = RuntimeParams.load(this);
        // Before the camera opens: the rate is part of the capture session, and a switch afterwards
        // costs a session rebuild.
        CameraHandler.setTargetFps(RuntimeParams.normalizeCameraFps(params.cameraFps));
        setupParamsPanel();
        applyParamsToOverlay();
        AdasAppHandler.applyLaneKeepParams(params);

        if (AdasConfig.visionSupercomboEnabled(this)) {
            try {
                visionPipeline = new VisionPipeline(this, laneOverlay, true, params.modelRunner);
                cameraHandler.setVisionPipeline(visionPipeline);
                applyParamsToVision();
                Log.i(LOG_TAG, "VisionPipeline ready: " + params.modelRunner + " + calib warp");
            } catch (Exception e) {
                Log.e(LOG_TAG, "VisionPipeline init failed", e);
                Toast.makeText(this, "ONNX init failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
            }
        } else {
            Log.i(LOG_TAG, "vision_supercombo disabled in config — skipping ONNX");
        }

        if (AdasConfig.visionTrafficEnabled(this)) {
            try {
                trafficVisionPipeline = new adas.app.vision.TrafficVisionPipeline(this, laneOverlay);
                cameraHandler.setTrafficVisionPipeline(trafficVisionPipeline);
                laneOverlay.setTrafficHudEnabled(true);
                Log.i(LOG_TAG, "TrafficVisionPipeline ready (lights)");
            } catch (Exception e) {
                Log.e(LOG_TAG,
                        "TrafficVisionPipeline init failed (put traffic_yolo.onnx in assets"
                                + " or /sdcard/adas_models/)", e);
                Toast.makeText(this, "Traffic YOLO missing: " + e.getMessage(), Toast.LENGTH_LONG).show();
            }
        } else {
            Log.i(LOG_TAG, "vision_traffic disabled (or both signs/lights off) — skipping YOLO");
        }

        mImageView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(android.graphics.SurfaceTexture surface, int w, int h) {
                Log.i(LOG_TAG, "TextureView available " + w + "x" + h);
                startCameraIfNeeded(w, h);
            }

            @Override
            public void onSurfaceTextureSizeChanged(android.graphics.SurfaceTexture surface, int w, int h) {
                cameraHandler.configurePreviewTransform(w, h);
            }

            @Override
            public boolean onSurfaceTextureDestroyed(android.graphics.SurfaceTexture surface) {
                Log.i(LOG_TAG, "TextureView destroyed — releasing camera");
                stopCamera();
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(android.graphics.SurfaceTexture surface) {}
        });

        requestRuntimePermissionsIfNeeded();
        updateLoggingButtonUi(Logger.getInstance().isRunning());

        loggingToggleButton.setOnClickListener(v -> {
            if (Logger.getInstance().isRunning()) {
                Logger.getInstance().stop();
                updateLoggingButtonUi(false);
                Toast.makeText(MainActivity.this, "Logging stopped", Toast.LENGTH_SHORT).show();
                return;
            }
            Log.i(LOG_TAG, "Starting logging...");
            maybePromptStorageAccess();
            startCameraIfNeeded(mImageView.getWidth(), mImageView.getHeight());

            File externalDir = Environment.getExternalStorageDirectory();
            File logsDir = new File(externalDir, "adas_logs");
            Logger.getInstance().start(logsDir.getAbsolutePath());
            if (cameraHandler != null) {
                cameraHandler.ensureBagIntrinsicsLogged();
            }
            updateLoggingButtonUi(true);
            Toast.makeText(MainActivity.this, "Logging started", Toast.LENGTH_SHORT).show();
        });

        paramsToggleButton.setOnClickListener(v -> setParamsOpen(!paramsOpen));
        findViewById(R.id.paramsResetButton).setOnClickListener(v -> {
            params = RuntimeParams.load(this);
            File cfg = RuntimeParams.configFile(this);
            if (cfg.exists()) {
                try {
                    if (cfg.delete()) {
                        params = RuntimeParams.load(this);
                    }
                } catch (Exception ignored) {
                }
            }
            bindSlidersFromParams();
            applyParamsToOverlay();
            applyParamsToVision();
            AdasAppHandler.applyLaneKeepParams(params);
            Toast.makeText(this, "Params reset from assets", Toast.LENGTH_SHORT).show();
        });
        findViewById(R.id.paramsCalibrateIntrinsicsButton).setOnClickListener(v -> toggleLensCalibration());
        findViewById(R.id.paramsSaveButton).setOnClickListener(v -> {
            readSlidersIntoParams();
            try {
                params.save(this);
                applyParamsToOverlay();
                applyParamsToVision();
                AdasAppHandler.applyLaneKeepParams(params);
                Toast.makeText(this, "Saved (native lane-keep updated if running)", Toast.LENGTH_LONG).show();
            } catch (Exception e) {
                Log.e(LOG_TAG, "save params failed", e);
                Toast.makeText(this, "Save failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
            }
        });
    }

    private void setupParamsPanel() {
        recordImagesCheck = findViewById(R.id.recordImagesCheck);
        recordImagesCheck.setOnCheckedChangeListener((v, checked) -> {
            params.recordCameraImages = checked;
            CameraHandler.setRecordCameraImages(checked);
            Toast.makeText(this, checked ? "Camera frames → bag" : "Camera frames NOT recorded",
                    Toast.LENGTH_SHORT).show();
        });
        laneKeepControllerGroup = findViewById(R.id.laneKeepControllerGroup);

        laneKeepControllerGroup.setOnCheckedChangeListener((group, checkedId) -> {
            if (suppressControllerUi) {
                return;
            }
            if (checkedId == R.id.laneKeepMpc) {
                params.laneKeepController = "mpc";
            } else if (checkedId == R.id.laneKeepFlowpilot) {
                params.laneKeepController = "fp";
            } else {
                params.laneKeepController = "pp";
            }
            AdasAppHandler.applyLaneKeepParams(params);
            Toast.makeText(this,
                    "Lane keep → " + params.laneKeepController.toUpperCase(),
                    Toast.LENGTH_SHORT).show();
        });

        modelRunnerGroup = findViewById(R.id.modelRunnerGroup);
        modelRunnerGroup.setOnCheckedChangeListener((group, checkedId) -> {
            if (suppressControllerUi) {
                return;
            }
            params.modelRunner = checkedId == R.id.modelThneed ? "thneed" : "onnx";
            rebuildVisionPipeline();
        });

        cameraFpsGroup = findViewById(R.id.cameraFpsGroup);
        cameraFpsGroup.setOnCheckedChangeListener((group, checkedId) -> {
            if (suppressControllerUi) {
                return;
            }
            params.cameraFps = checkedId == R.id.fps30
                    ? CameraHandler.FPS_FAST : CameraHandler.FPS_MODEL;
            applyCameraFps();
        });

        carSpinner = findViewById(R.id.carSpinner);
        final String[] carLabels = new String[AdasConfig.CARS.length];
        for (int i = 0; i < AdasConfig.CARS.length; i++) {
            carLabels[i] = AdasConfig.CARS[i][1];
        }
        android.widget.ArrayAdapter<String> carAdapter = new android.widget.ArrayAdapter<>(
                this, android.R.layout.simple_spinner_item, carLabels);
        carAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        carSpinner.setAdapter(carAdapter);
        carSpinner.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(android.widget.AdapterView<?> parent, View view, int position, long id) {
                if (suppressControllerUi) {
                    return;
                }
                final String name = AdasConfig.CARS[position][0];
                // Written at once: the make decides which DBC parses the bus and what the car's
                // parameters are, and that is picked up when the service starts rather than on the
                // fly. Nothing here changes mid-drive — there is one car under us on the road.
                try {
                    RuntimeParams.setCarName(getApplicationContext(), name);
                    Toast.makeText(MainActivity.this, "Car: " + name + " — restart the ADAS service to apply",
                            Toast.LENGTH_LONG).show();
                } catch (Exception e) {
                    Log.e(LOG_TAG, "saving the car name failed", e);
                    Toast.makeText(MainActivity.this, "Could not save the car: " + e.getMessage(),
                            Toast.LENGTH_LONG).show();
                }
            }

            @Override
            public void onNothingSelected(android.widget.AdapterView<?> parent) {}
        });

        bindSlidersFromParams();
    }

    /** \brief Apply the chosen capture rate: the AE range lives in the session, so the camera restarts. */
    private void applyCameraFps() {
        CameraHandler.setTargetFps(RuntimeParams.normalizeCameraFps(params.cameraFps));
        try {
            params.save(this);
        } catch (Exception e) {
            Log.e(LOG_TAG, "save params failed", e);
        }
        stopCamera();
        if (mImageView != null && mImageView.isAvailable()) {
            startCameraIfNeeded(mImageView.getWidth(), mImageView.getHeight());
        }
        Toast.makeText(this, "Camera " + CameraHandler.getTargetFps() + " fps", Toast.LENGTH_SHORT).show();
    }

    private void rebuildVisionPipeline() {
        final String choice = RuntimeParams.normalizeModelRunner(params.modelRunner);
        Toast.makeText(this, "Model -> " + choice.toUpperCase() + ", rebuilding...",
                Toast.LENGTH_SHORT).show();
        visionRebuild.execute(() -> {
            VisionPipeline old = visionPipeline;
            try {
                cameraHandler.setVisionPipeline(null);
                visionPipeline = null;
                if (old != null) {
                    old.close();
                }
                VisionPipeline fresh = new VisionPipeline(this, laneOverlay, true, choice);
                visionPipeline = fresh;
                cameraHandler.setVisionPipeline(fresh);
                runOnUiThread(() -> {
                    applyParamsToVision();
                    Toast.makeText(this, "Model: " + choice.toUpperCase(), Toast.LENGTH_SHORT).show();
                });
                Log.i(LOG_TAG, "VisionPipeline rebuilt: " + choice);
            } catch (Exception e) {
                Log.e(LOG_TAG, "vision pipeline rebuild failed", e);
                runOnUiThread(() -> Toast.makeText(this,
                        "Model switch failed: " + e.getMessage(), Toast.LENGTH_LONG).show());
            }
        });
    }

    private boolean suppressControllerUi;

    /** Lay the parameters out over the panel's widgets. */
    private void bindSlidersFromParams() {
        if (recordImagesCheck != null) {
            recordImagesCheck.setChecked(params.recordCameraImages);
            CameraHandler.setRecordCameraImages(params.recordCameraImages);
        }
        if (modelRunnerGroup != null) {
            suppressControllerUi = true;
            modelRunnerGroup.check("thneed".equals(RuntimeParams.normalizeModelRunner(params.modelRunner))
                    ? R.id.modelThneed : R.id.modelOnnx);
            suppressControllerUi = false;
        }
        if (cameraFpsGroup != null) {
            suppressControllerUi = true;
            cameraFpsGroup.check(RuntimeParams.normalizeCameraFps(params.cameraFps) == CameraHandler.FPS_FAST
                    ? R.id.fps30 : R.id.fps20);
            suppressControllerUi = false;
        }
        if (carSpinner != null) {
            suppressControllerUi = true;
            final String current = AdasConfig.carName(getApplicationContext());
            int index = 0;
            for (int i = 0; i < AdasConfig.CARS.length; i++) {
                if (AdasConfig.CARS[i][0].equals(current)) {
                    index = i;
                    break;
                }
            }
            carSpinner.setSelection(index);
            suppressControllerUi = false;
        }
        if (laneKeepControllerGroup != null) {
            String c = RuntimeParams.normalizeController(params.laneKeepController);
            suppressControllerUi = true;
            int id = R.id.laneKeepPp;
            if ("mpc".equals(c)) {
                id = R.id.laneKeepMpc;
            } else if ("fp".equals(c)) {
                id = R.id.laneKeepFlowpilot;
            }
            laneKeepControllerGroup.check(id);
            suppressControllerUi = false;
        }
    }

    /** Read back whatever the panel still holds. */
    private void readSlidersIntoParams() {
        if (recordImagesCheck != null) {
            params.recordCameraImages = recordImagesCheck.isChecked();
        }
    }

    private void applyParamsToOverlay() {
        if (laneOverlay == null) {
            return;
        }
        laneOverlay.setCameraHeight(params.heightM);
        laneOverlay.setCalibRpyDeg(params.rollDeg, params.pitchDeg, params.yawDeg);
        int w = cameraHandler != null ? cameraHandler.W : params.calibWidth;
        int h = cameraHandler != null ? cameraHandler.H : params.calibHeight;
        int calibW = Math.max(1, params.calibWidth);
        int calibH = Math.max(1, params.calibHeight);
        float sx = w / (float) calibW;
        float sy = h / (float) calibH;
        laneOverlay.setIntrinsics(
                params.fx * sx, params.fy * sy, params.cx * sx, params.cy * sy, w, h);
    }

    /** Hand vision its calibration, preferring the focal length measured by the camera itself. */
    private void applyParamsToVision() {
        if (visionPipeline == null) {
            return;
        }
        float fx = params.fx;
        float fy = params.fy;
        float cx = params.cx;
        float cy = params.cy;
        int calibW = params.calibWidth;
        int calibH = params.calibHeight;

        final float measured = cameraHandler != null ? cameraHandler.measuredFocalPx() : 0f;
        // Our own prior beats the camera: it came from a chessboard on this very phone. A foreign
        // one is worse than the camera: it is just a number from a different lens.
        final boolean priorIsForThisPhone = !params.intrinsicsPriorDevice.isEmpty()
                && params.intrinsicsPriorDevice.equalsIgnoreCase(android.os.Build.MODEL);
        if (params.intrinsicsFromDevice && !priorIsForThisPhone && measured > 1f && cameraHandler != null) {
            fx = measured;
            fy = measured;
            cx = cameraHandler.W * 0.5f;
            cy = cameraHandler.H * 0.5f;
            calibW = cameraHandler.W;
            calibH = cameraHandler.H;
            Log.i(LOG_TAG, String.format(java.util.Locale.US,
                    "intrinsics from the camera: f=%.1f px at %dx%d (%s); prior %.1f was taken on "
                            + "'%s' and this is '%s' — taking the camera",
                    measured, calibW, calibH, cameraHandler.measuredFocalSource(), params.fx,
                    params.intrinsicsPriorDevice.isEmpty() ? "unknown" : params.intrinsicsPriorDevice,
                    android.os.Build.MODEL));
        } else {
            final String why = priorIsForThisPhone
                    ? "the prior was taken on this phone — more accurate than the camera's datasheet estimate"
                    : (params.intrinsicsFromDevice ? "the camera reported no focal length"
                                                   : "intrinsics_from_device is off");
            Log.i(LOG_TAG, String.format(java.util.Locale.US,
                    "intrinsics from the config: f=%.1f at %dx%d (%s)", fx, calibW, calibH, why));
        }

        visionPipeline.setCalib(
                params.rollDeg, params.pitchDeg, params.yawDeg, fx, fy, cx, cy, calibW, calibH);
    }

    private void setParamsOpen(boolean open) {
        paramsOpen = open;
        paramsPanel.setVisibility(open ? View.VISIBLE : View.GONE);
        if (open) {
            bindSlidersFromParams();
        }
    }

    private void updateCanOnlineUi() {
        long last = ZMQBridgeService.lastPandaHealthElapsedMs();
        boolean online = last > 0 && (SystemClock.elapsedRealtime() - last) <= CAN_ONLINE_TIMEOUT_MS;
        canOnlineLed.setBackgroundResource(online ? R.drawable.bg_status_led_on : R.drawable.bg_status_led_off);
        canOnlineText.setText(online ? R.string.can_online : R.string.can_offline);
        canOnlineText.setTextColor(online ? 0xFF2ECC71 : 0x80FFFFFF);
    }

    private void onOutboundMessage(String topic, ZMQMessage message) {
        if (laneOverlay == null || message == null) {
            return;
        }
        if (message.hasLaneKeep()) {
            LaneKeepOuter.LaneKeepState lk = message.getLaneKeep();
            laneOverlay.setLaneKeep(
                    lk.getHasTarget(),
                    (float) lk.getTargetX(),
                    (float) lk.getTargetY(),
                    (float) lk.getLookaheadM(),
                    (float) lk.getCurvature(),
                    (float) lk.getSteerRad(),
                    lk.getStatus());
            laneOverlay.setTorqueSaturated(lk.getTorqueSaturated());
        }
        // vision/path: the reference line the lateral loop drives on, built in C++ by the Planner.
        // Drawn instead of the model plan, which was one input to it rather than the result.
        if (message.hasLanePath()) {
            LanePathOuterClass.LanePath lp = message.getLanePath();
            final int n = lp.getPolylineCount();
            if (n >= 2) {
                float[] xs = new float[n];
                float[] ys = new float[n];
                for (int i = 0; i < n; i++) {
                    xs[i] = (float) lp.getPolyline(i).getX();
                    ys[i] = (float) lp.getPolyline(i).getY();
                }
                laneOverlay.setCenterline(xs, ys, lp.getLaneAnchored());
            } else {
                laneOverlay.clearCenterline();
            }
        }
        if (message.hasSteerCommand()) {
            SteerOuter.SteerCommand cmd = message.getSteerCommand();
            laneOverlay.setSteerCommand(cmd.getTorqueCnm(), cmd.getEnabled());
        }
        // panda/health used to feed an HCA status line here; that HUD was taken off the screen, and
        // the same gates are visible in the bag and in `Panda health:` logcat lines.
        // CAN speed, an input of the 0.9.x model. Arrives at 100 Hz, far more often than frames, so the
        // latest value is simply kept and the frame gets whatever it was at inference time.
        if (message.hasCarState()) {
            final float vEgo = message.getCarState().getVEgo();
            if (visionPipeline != null) {
                visionPipeline.setEgoSpeed(vEgo);
            }
            // The speed readout is the driver's, not the model's: it comes from CAN, at 100 Hz.
            laneOverlay.setEgoSpeed(vEgo);
            // Same value gates the bag: standing still writes no JPEGs.
            CameraHandler.setEgoSpeed(vEgo);
        }
        if (message.hasSafetyWarn()) {
            SafetyWarnOuter.SafetyWarnState w = message.getSafetyWarn();
            laneOverlay.setSafetyWarn(w.getFcw(), w.getAeb(), w.getLldw(), w.getRldw());
        }
        if (message.hasTrafficVision()) {
            TrafficVisionOuter.TrafficVisionState tv = message.getTrafficVision();
            laneOverlay.setTrafficVision(tv.getTflColorValue(), tv.getTflConf());
        }
        if (message.hasCameraCalib()) {
            CameraCalibOuter.CameraCalibrationState c = message.getCameraCalib();
            if (c.getCalibrationSuccess() || c.getCalPercent() >= 50) {
                params.rollDeg = (float) c.getRollDeg();
                params.pitchDeg = (float) c.getPitchDeg();
                params.yawDeg = (float) c.getYawDeg();
                if (c.getCameraHeightM() > 0.2) {
                    params.heightM = (float) c.getCameraHeightM();
                }
                if (c.getFx() > 1) {
                    params.fx = (float) c.getFx();
                    params.fy = (float) c.getFy();
                    params.cx = (float) c.getCx();
                    params.cy = (float) c.getCy();
                }
                applyParamsToOverlay();
                applyParamsToVision();
                if (paramsOpen) {
                    bindSlidersFromParams();
                }
                persistCalibrationIfSettled(c.getCalibrationSuccess());
            }
        }
    }

    /** Keep a converged calibration across restarts. */
    /** Start or stop measuring the lens against a printed chessboard. */
    private void toggleLensCalibration() {
        if (intrinsicsCalibrator != null) {
            stopLensCalibration("Lens calibration cancelled");
            return;
        }
        if (cameraHandler == null) {
            Toast.makeText(this, "Camera is not running", Toast.LENGTH_SHORT).show();
            return;
        }
        if (!org.opencv.android.OpenCVLoader.initLocal()) {
            Toast.makeText(this, "OpenCV failed to load — calibration unavailable", Toast.LENGTH_LONG).show();
            Log.e(LOG_TAG, "OpenCVLoader.initLocal() returned false");
            return;
        }

        intrinsicsCalibrator = new adas.app.vision.IntrinsicsCalibrator();
        calibFrameCounter = 0;
        calibDetect = java.util.concurrent.Executors.newSingleThreadExecutor(
                r -> new Thread(r, "LensCalibrate"));
        if (laneOverlay != null) {
            laneOverlay.setCalibrationActive(true);
            laneOverlay.setCalibrationProgress(0, adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS,
                    "Show the board, then move it between shots");
        }
        // The parameter panel covers half the frame — the chessboard cannot be caught with it open.
        if (paramsOpen) {
            setParamsOpen(false);
        }
        // The button is the only way a user reaches chessboard capture: it restarts the camera with
        // autofocus (locked once settled) and the streams pinned to the physical main module. Point
        // the phone at the board from the working distance before pressing, so the focus locks on
        // the board and not on whatever else was in front of the lens.
        cameraHandler.setChessboardCaptureMode(true);
        cameraHandler.setFrameTap(new CameraHandler.FrameTap() {
            @Override
            public boolean wantsFrame() {
                return intrinsicsCalibrator != null && !calibBusy.get()
                        && (++calibFrameCounter % CALIB_DETECT_EVERY_N == 0);
            }

            @Override
            public void onFrame(adas.app.vision.YuvFrame yuv, long captureTsMs) {
                onCalibrationFrame(yuv);
            }
        });
        Toast.makeText(this, "Show the chessboard and move it between shots", Toast.LENGTH_LONG).show();
    }

    private void onCalibrationFrame(adas.app.vision.YuvFrame yuv) {
        final adas.app.vision.IntrinsicsCalibrator calibrator = intrinsicsCalibrator;
        if (calibrator == null || !calibBusy.compareAndSet(false, true)) {
            return;
        }
        final java.util.concurrent.ExecutorService executor = calibDetect;
        if (executor == null || executor.isShutdown()) {
            calibBusy.set(false);
            return;
        }
        executor.execute(() -> {
            try {
                adas.app.vision.IntrinsicsCalibrator.Detection d =
                        calibrator.offer(yuv.y, yuv.width, yuv.height);
                final int kept = calibrator.keptViews();
                final boolean full = calibrator.isFull();
                runOnUiThread(() -> {
                    if (intrinsicsCalibrator != calibrator) {
                        return;
                    }
                    if (laneOverlay != null) {
                        laneOverlay.setCalibrationPoints(d.corners, d.accepted);
                        final String hint;
                        if (d.corners == null) {
                            hint = "Board not visible";
                        } else if (d.accepted) {
                            hint = "Kept — move the board";
                        } else {
                            hint = "Too close to a shot already taken";
                        }
                        laneOverlay.setCalibrationProgress(kept,
                                adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS, hint);
                    }
                    if (full) {
                        finishLensCalibration(calibrator);
                    }
                });
            } catch (Throwable t) {
                Log.e(LOG_TAG, "lens calibration frame failed", t);
            } finally {
                calibBusy.set(false);
            }
        });
    }

    private void finishLensCalibration(adas.app.vision.IntrinsicsCalibrator calibrator) {
        if (cameraHandler != null) {
            cameraHandler.setFrameTap(null);
        }
        if (laneOverlay != null) {
            laneOverlay.setCalibrationProgress(adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS,
                    adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS, "Solving…");
        }
        final java.util.concurrent.ExecutorService executor = calibDetect;
        if (executor == null) {
            stopLensCalibration("Lens calibration stopped");
            return;
        }
        executor.execute(() -> {
            final adas.app.vision.IntrinsicsCalibrator.Result r = calibrator.solve();
            runOnUiThread(() -> {
                if (r.ok) {
                    params.fx = (float) r.fx;
                    params.fy = (float) r.fy;
                    params.cx = (float) r.cx;
                    params.cy = (float) r.cy;
                    params.calibWidth = r.width;
                    params.calibHeight = r.height;
                    // Measured on this very phone — stamp it, or the prior reads as foreign next start.
                    params.intrinsicsPriorDevice = android.os.Build.MODEL;
                    try {
                        params.save(getApplicationContext());
                    } catch (Exception e) {
                        Log.e(LOG_TAG, "saving measured intrinsics failed", e);
                    }
                    applyParamsToOverlay();
                    applyParamsToVision();
                    if (paramsOpen) {
                        bindSlidersFromParams();
                    }
                    showCalibrationResult(String.format(java.util.Locale.US,
                            "Saved: fx=%.0f fy=%.0f cx=%.0f cy=%.0f, %.2f px",
                            r.fx, r.fy, r.cx, r.cy, r.reprojectionPx));
                } else {
                    showCalibrationResult("Rejected: " + r.message);
                }
            });
        });
    }

    /** Hold the outcome on screen for a few seconds before the overlay goes back to the road. */
    private void showCalibrationResult(String message) {
        if (laneOverlay != null) {
            laneOverlay.setCalibrationProgress(adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS,
                    adas.app.vision.IntrinsicsCalibrator.TARGET_VIEWS, message);
        }
        Log.i(LOG_TAG, "lens calibration: " + message);
        uiHandler.postDelayed(() -> stopLensCalibration(message), 6000L);
    }

    private void stopLensCalibration(String message) {
        if (cameraHandler != null) {
            cameraHandler.setFrameTap(null);
            // Back to the driving focus policy (infinity, no AF) the moment calibration ends.
            cameraHandler.setChessboardCaptureMode(false);
        }
        if (laneOverlay != null) {
            laneOverlay.setCalibrationActive(false);
        }
        adas.app.vision.IntrinsicsCalibrator calibrator = intrinsicsCalibrator;
        intrinsicsCalibrator = null;
        if (calibrator != null) {
            calibrator.clear();
        }
        if (calibDetect != null) {
            calibDetect.shutdown();
            calibDetect = null;
        }
        calibBusy.set(false);
        Toast.makeText(this, message, Toast.LENGTH_LONG).show();
    }

    private void persistCalibrationIfSettled(boolean converged) {
        if (!converged) {
            return;
        }
        final long now = android.os.SystemClock.elapsedRealtime();
        if (lastCalibSaveMs != 0L && now - lastCalibSaveMs < CALIB_SAVE_MIN_INTERVAL_MS) {
            return;
        }
        if (!Float.isNaN(savedRollDeg)
                && Math.abs(params.rollDeg - savedRollDeg) < CALIB_SAVE_MIN_DELTA_DEG
                && Math.abs(params.pitchDeg - savedPitchDeg) < CALIB_SAVE_MIN_DELTA_DEG
                && Math.abs(params.yawDeg - savedYawDeg) < CALIB_SAVE_MIN_DELTA_DEG) {
            return;
        }
        // Do not save a calibration learned blind.
        if (visionPipeline != null && !visionPipeline.seesRoad()) {
            return;
        }
        lastCalibSaveMs = now;
        savedRollDeg = params.rollDeg;
        savedPitchDeg = params.pitchDeg;
        savedYawDeg = params.yawDeg;
        final float roll = savedRollDeg;
        final float pitch = savedPitchDeg;
        final float yaw = savedYawDeg;
        calibSave.execute(() -> {
            try {
                params.save(getApplicationContext());
                Log.i(LOG_TAG, String.format(
                        "calibration saved: roll=%.2f pitch=%.2f yaw=%.2f", roll, pitch, yaw));
            } catch (Exception e) {
                Log.e(LOG_TAG, "saving the learned calibration failed", e);
            }
        });
    }

    private void updateLoggingButtonUi(boolean logging) {
        if (loggingToggleButton == null) {
            return;
        }
        loggingToggleButton.setImageResource(logging ? R.drawable.ic_log_stop : R.drawable.ic_log_rec);
        loggingToggleButton.setBackgroundResource(
                logging ? R.drawable.bg_log_fab_recording : R.drawable.bg_log_fab);
        loggingToggleButton.setSelected(logging);
        loggingToggleButton.setContentDescription(
                logging ? getString(R.string.log_toggle_stop) : getString(R.string.log_toggle_start));
    }

    @Override
    protected void onResume() {
        super.onResume();
        updateLoggingButtonUi(Logger.getInstance().isRunning());
        ZMQBridgeService.addOutboundListener(outboundListener);
        uiHandler.post(canStatusTick);
        if (gpsHandler != null) {
            gpsHandler.start();
        }
        if (imuHandler != null) {
            imuHandler.start();
        }
        if (phoneStatsHandler != null) {
            phoneStatsHandler.start();
        }
        if (mImageView != null && mImageView.isAvailable()) {
            startCameraIfNeeded(mImageView.getWidth(), mImageView.getHeight());
        }
    }

    @Override
    protected void onPause() {
        uiHandler.removeCallbacks(canStatusTick);
        ZMQBridgeService.removeOutboundListener(outboundListener);
        stopCamera();
        if (gpsHandler != null) {
            gpsHandler.stop();
        }
        if (imuHandler != null) {
            imuHandler.stop();
        }
        if (phoneStatsHandler != null) {
            phoneStatsHandler.stop();
        }
        super.onPause();
    }

    private void requestRuntimePermissionsIfNeeded() {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                        != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION)
                        != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                        != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                        != PackageManager.PERMISSION_GRANTED
                || (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
                        && ContextCompat.checkSelfPermission(
                                        this, Manifest.permission.HIGH_SAMPLING_RATE_SENSORS)
                                != PackageManager.PERMISSION_GRANTED)) {
            java.util.ArrayList<String> perms = new java.util.ArrayList<>();
            perms.add(Manifest.permission.CAMERA);
            perms.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
            perms.add(Manifest.permission.ACCESS_FINE_LOCATION);
            perms.add(Manifest.permission.ACCESS_COARSE_LOCATION);
            // Microphone, for the audio recorded alongside the bag; a refusal costs only the audio.
            perms.add(Manifest.permission.RECORD_AUDIO);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                perms.add(Manifest.permission.HIGH_SAMPLING_RATE_SENSORS);
            }
            ActivityCompat.requestPermissions(this, perms.toArray(new String[0]), REQ_PERMISSIONS);
        }
    }

    private void maybePromptStorageAccess() {
        if (storagePromptShown) {
            return;
        }
        if (Build.VERSION.SDK_INT >= 30 && !Environment.isExternalStorageManager()) {
            storagePromptShown = true;
            Toast.makeText(this, "Grant All files access for bag logging", Toast.LENGTH_LONG).show();
            try {
                startActivity(new Intent(Settings.ACTION_MANAGE_ALL_FILES_ACCESS_PERMISSION));
            } catch (Exception e) {
                Log.e(LOG_TAG, "Cannot open storage settings", e);
            }
        }
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQ_PERMISSIONS) {
            return;
        }
        boolean cameraOk = checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
        Log.i(LOG_TAG, "Permissions result: cameraOk=" + cameraOk);
        if (cameraOk && mImageView != null && mImageView.isAvailable()) {
            startCameraIfNeeded(mImageView.getWidth(), mImageView.getHeight());
        } else if (!cameraOk) {
            Toast.makeText(this, "Camera permission required", Toast.LENGTH_LONG).show();
        }
        if (gpsHandler != null) {
            gpsHandler.start();
        }
        if (imuHandler != null) {
            imuHandler.start();
        }
        if (phoneStatsHandler != null) {
            phoneStatsHandler.start();
        }
    }

    private void startCameraIfNeeded(int w, int h) {
        if (cameraHandler == null || mImageView == null) {
            return;
        }
        if (w > 0 && h > 0) {
            cameraHandler.configurePreviewTransform(w, h);
        }
        if (cameraStarted && cameraHandler.isCameraOpen()) {
            return;
        }
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            Log.w(LOG_TAG, "Camera permission not granted yet");
            return;
        }
        if (!mImageView.isAvailable()) {
            Log.w(LOG_TAG, "TextureView surface not available yet");
            return;
        }
        if (cameraStarted && !cameraHandler.isCameraOpen()) {
            Log.w(LOG_TAG, "Camera marked started but not open — restarting");
            cameraHandler.stop();
            cameraStarted = false;
        }
        Log.i(LOG_TAG, "Starting camera preview " + w + "x" + h);
        cameraHandler.start();
        cameraStarted = true;
    }

    private void stopCamera() {
        if (cameraHandler != null) {
            cameraHandler.stop();
        }
        cameraStarted = false;
        // Off the road: no camera, no vision, so nothing left that could clear an alert. A tone that
        // keeps sounding after the pipeline stopped is the same lie as a frozen overlay.
        if (laneOverlay != null) {
            laneOverlay.stopSounds();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        uiHandler.removeCallbacks(canStatusTick);
        ZMQBridgeService.removeOutboundListener(outboundListener);
        if (visionPipeline != null) {
            visionPipeline.close();
        }
        visionRebuild.shutdown();
        if (trafficVisionPipeline != null) {
            trafficVisionPipeline.close();
            trafficVisionPipeline = null;
        }
        if (cameraHandler != null) {
            cameraHandler.release();
            cameraStarted = false;
        }
        if (gpsHandler != null) {
            gpsHandler.stop();
        }
        if (imuHandler != null) {
            imuHandler.stop();
        }
        if (phoneStatsHandler != null) {
            phoneStatsHandler.stop();
            phoneStatsHandler = null;
        }
        stopService(new Intent(getApplicationContext(), AdasAppHandler.class));
        Log.i(LOG_TAG, "All services stopped in onDestroy");
    }
}
