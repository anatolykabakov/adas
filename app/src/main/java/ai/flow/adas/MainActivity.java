package ai.flow.adas;

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

import ai.flow.adas.Messages.ZMQMessage;
import ai.flow.adas.vision.LaneOverlayView;
import ai.flow.adas.vision.VisionPipeline;

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
    private ai.flow.adas.vision.TrafficVisionPipeline trafficVisionPipeline;

    private boolean cameraStarted;
    private boolean storagePromptShown;
    private boolean paramsOpen;

    private RuntimeParams params = new RuntimeParams();
    private ParamSlider rollSlider;
    private ParamSlider pitchSlider;
    private ParamSlider yawSlider;
    private ParamSlider heightSlider;
    private ParamSlider camXSlider;
    private ParamSlider camYSlider;
    private android.widget.CheckBox recordImagesCheck;
    private ParamSlider ppKddSlider;
    private ParamSlider ppLdMinSlider;
    private ParamSlider ppLdMaxSlider;
    private ParamSlider wheelbaseSlider;
    private ParamSlider ppShiftSlider;
    private ParamSlider steerRatioSlider;
    private ParamSlider laneBlendSlider;
    private RadioGroup laneKeepControllerGroup;
    private RadioGroup modelRunnerGroup;

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

        loggingToggleButton = findViewById(R.id.loggingToggleButton);
        paramsToggleButton = findViewById(R.id.paramsToggleButton);
        canOnlineLed = findViewById(R.id.canOnlineLed);
        canOnlineText = findViewById(R.id.canOnlineText);
        paramsPanel = findViewById(R.id.paramsPanel);
        mImageView = findViewById(R.id.textureView);
        laneOverlay = findViewById(R.id.laneOverlay);

        cameraHandler = new CameraHandler(getApplication().getApplicationContext(), mImageView);
        cameraHandler.setFailureListener(reason -> uiHandler.post(() -> {
            stopCamera();
            if (laneOverlay != null) {
                laneOverlay.setHcaStatus("Camera dead: " + reason);
            }
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
                trafficVisionPipeline = new ai.flow.adas.vision.TrafficVisionPipeline(this, laneOverlay);
                cameraHandler.setTrafficVisionPipeline(trafficVisionPipeline);
                Log.i(LOG_TAG, "TrafficVisionPipeline ready (~3 Hz YOLO)"
                        + " signs=" + AdasConfig.visionTrafficSignsEnabled(this)
                        + " lights=" + AdasConfig.visionTrafficLightsEnabled(this));
            } catch (Exception e) {
                Log.e(LOG_TAG, "TrafficVisionPipeline init failed (put traffic_yolo.onnx in assets or /sdcard/adas_models/)", e);
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
        rollSlider = new ParamSlider(findViewById(R.id.rowRoll), "Roll", "°", -15f, 15f, 1);
        pitchSlider = new ParamSlider(findViewById(R.id.rowPitch), "Pitch", "°", -20f, 10f, 1);
        yawSlider = new ParamSlider(findViewById(R.id.rowYaw), "Yaw", "°", -20f, 20f, 1);
        heightSlider = new ParamSlider(findViewById(R.id.rowHeight), "Height", "m", 0.40f, 2.20f, 2);
        camXSlider = new ParamSlider(findViewById(R.id.rowCamX), "X", "m", 0f, 3f, 2);
        camYSlider = new ParamSlider(findViewById(R.id.rowCamY), "Y", "m", -1f, 1f, 2);
        recordImagesCheck = findViewById(R.id.recordImagesCheck);
        recordImagesCheck.setOnCheckedChangeListener((v, checked) -> {
            params.recordCameraImages = checked;
            CameraHandler.setRecordCameraImages(checked);
            Toast.makeText(this, checked ? "Camera frames → bag" : "Camera frames NOT recorded",
                    Toast.LENGTH_SHORT).show();
        });
        ppKddSlider = new ParamSlider(findViewById(R.id.rowPpKdd), "K_dd", "", 0.05f, 1.5f, 2);
        ppLdMinSlider = new ParamSlider(findViewById(R.id.rowPpLdMin), "Ld min", "m", 1f, 30f, 1);
        ppLdMaxSlider = new ParamSlider(findViewById(R.id.rowPpLdMax), "Ld max", "m", 1f, 40f, 1);
        wheelbaseSlider = new ParamSlider(findViewById(R.id.rowWheelbase), "Wheelbase", "m", 2.0f, 3.2f, 3);
        ppShiftSlider = new ParamSlider(findViewById(R.id.rowPpShift), "Shift", "m", 0f, 3f, 2);
        steerRatioSlider = new ParamSlider(findViewById(R.id.rowSteerRatio), "Steer ratio", "", 8f, 25f, 1);
        laneBlendSlider = new ParamSlider(findViewById(R.id.rowLaneBlend), "Lane blend", "", 0f, 1f, 2);
        laneKeepControllerGroup = findViewById(R.id.laneKeepControllerGroup);

        ParamSlider.Listener liveOverlay = v -> {
            readSlidersIntoParams();
            applyParamsToOverlay();
        };
        ParamSlider.Listener liveWarp = v -> {
            readSlidersIntoParams();
            applyParamsToOverlay();
            applyParamsToVision();
        };
        ParamSlider.Listener livePp = v -> {
            readSlidersIntoParams();
            applyParamsToOverlay();
            AdasAppHandler.applyLaneKeepParams(params);
        };
        rollSlider.setListener(liveWarp);
        pitchSlider.setListener(liveWarp);
        yawSlider.setListener(liveWarp);
        heightSlider.setListener(liveOverlay);
        ppShiftSlider.setListener(livePp);
        ppKddSlider.setListener(livePp);
        ppLdMinSlider.setListener(livePp);
        ppLdMaxSlider.setListener(livePp);
        steerRatioSlider.setListener(livePp);
        laneBlendSlider.setListener(v -> {
            readSlidersIntoParams();
            AdasAppHandler.applyLaneKeepParams(params);
        });

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
            updatePpSlidersEnabled();
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

        bindSlidersFromParams();
    }

    /**
     * Rebuilds the vision pipeline for the selected model, live and without a restart.
     *
     * <p>Off the UI thread: ONNX init takes hundreds of milliseconds and thneed loads a 50 MB model,
     * which would be an ANR on the main thread. While it rebuilds the camera delivers to nobody, so
     * lateral control has no target for that moment — switch on a straight, not in a bend.
     */
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

    private void bindSlidersFromParams() {
        rollSlider.setValue(params.rollDeg);
        pitchSlider.setValue(params.pitchDeg);
        yawSlider.setValue(params.yawDeg);
        heightSlider.setValue(params.heightM);
        camXSlider.setValue(params.camX);
        camYSlider.setValue(params.camY);
        if (recordImagesCheck != null) {
            recordImagesCheck.setChecked(params.recordCameraImages);
            CameraHandler.setRecordCameraImages(params.recordCameraImages);
        }
        ppKddSlider.setValue(params.ppKdd);
        ppLdMinSlider.setValue(params.ppLdMin);
        ppLdMaxSlider.setValue(params.ppLdMax);
        wheelbaseSlider.setValue(params.wheelbaseM);
        ppShiftSlider.setValue(params.ppShift);
        steerRatioSlider.setValue(params.steerRatio);
        if (laneBlendSlider != null) {
            laneBlendSlider.setValue(params.laneBlendScale);
        }
        if (modelRunnerGroup != null) {
            suppressControllerUi = true;
            modelRunnerGroup.check("thneed".equals(RuntimeParams.normalizeModelRunner(params.modelRunner))
                    ? R.id.modelThneed : R.id.modelOnnx);
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
        updatePpSlidersEnabled();
    }

    private void updatePpSlidersEnabled() {
        boolean pp = "pp".equalsIgnoreCase(RuntimeParams.normalizeController(params.laneKeepController));
        if (ppKddSlider != null) {
            ppKddSlider.setEnabled(pp);
            ppLdMinSlider.setEnabled(pp);
            ppLdMaxSlider.setEnabled(pp);
            ppShiftSlider.setEnabled(pp);
        }
    }

    private void readSlidersIntoParams() {
        params.rollDeg = rollSlider.getValue();
        params.pitchDeg = pitchSlider.getValue();
        params.yawDeg = yawSlider.getValue();
        params.heightM = heightSlider.getValue();
        params.camX = camXSlider.getValue();
        params.camY = camYSlider.getValue();
        if (recordImagesCheck != null) {
            params.recordCameraImages = recordImagesCheck.isChecked();
        }
        params.ppKdd = ppKddSlider.getValue();
        params.ppLdMin = ppLdMinSlider.getValue();
        params.ppLdMax = ppLdMaxSlider.getValue();
        if (params.ppLdMin > params.ppLdMax) {
            float t = params.ppLdMin;
            params.ppLdMin = params.ppLdMax;
            params.ppLdMax = t;
            ppLdMinSlider.setValue(params.ppLdMin);
            ppLdMaxSlider.setValue(params.ppLdMax);
        }
        params.wheelbaseM = wheelbaseSlider.getValue();
        params.ppShift = ppShiftSlider.getValue();
        params.steerRatio = steerRatioSlider.getValue();
        if (laneBlendSlider != null) {
            params.laneBlendScale = laneBlendSlider.getValue();
        }
    }

    private void applyParamsToOverlay() {
        if (laneOverlay == null) {
            return;
        }
        laneOverlay.setCameraHeight(params.heightM);
        laneOverlay.setWaypointShift(params.ppShift);
        laneOverlay.setSteerRatio(params.steerRatio);
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

    private void applyParamsToVision() {
        if (visionPipeline == null) {
            return;
        }
        visionPipeline.setCalib(
                params.rollDeg, params.pitchDeg, params.yawDeg,
                params.fx, params.fy, params.cx, params.cy,
                params.calibWidth, params.calibHeight);
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
        if (message.hasSteerCommand()) {
            SteerOuter.SteerCommand cmd = message.getSteerCommand();
            laneOverlay.setSteerCommand(cmd.getTorqueCnm(), cmd.getEnabled());
        }
        if (message.hasPandaHealth()) {
            Panda.PandaHealth h = message.getPandaHealth();
            boolean ign = h.getIgnitionLine() || h.getIgnitionCan();
            String status;
            if (!ign) {
                status = "HCA blocked: no ignition";
            } else if (!h.getControlsAllowed()) {
                status = "HCA blocked: controls_allowed=0 (engage stock ACC)";
            } else if (h.getHeartbeatLost()) {
                status = "HCA warn: panda heartbeat lost";
            } else {
                status = String.format("HCA ok (safety=%d)", h.getSafetyMode());
            }
            laneOverlay.setHcaStatus(status);
        }
        // CAN speed, an input of the 0.9.x model. Arrives at 100 Hz, far more often than frames, so the
        // latest value is simply kept and the frame gets whatever it was at inference time.
        if (message.hasCarState() && visionPipeline != null) {
            visionPipeline.setEgoSpeed(message.getCarState().getVEgo());
        }
        if (message.hasSafetyWarn()) {
            SafetyWarnOuter.SafetyWarnState w = message.getSafetyWarn();
            laneOverlay.setSafetyWarn(w.getFcw(), w.getAeb(), w.getLldw(), w.getRldw());
        }
        if (message.hasTrafficVision()) {
            TrafficVisionOuter.TrafficVisionState tv = message.getTrafficVision();
            laneOverlay.setTrafficVision(
                    tv.getSpeedLimitKmh(),
                    tv.getVEgoKmh(),
                    tv.getOverspeed(),
                    tv.getOverspeedKmh(),
                    tv.getTflColorValue(),
                    tv.getTflConf(),
                    tv.getStatus());
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
            }
        }
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
