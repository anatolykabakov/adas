package adas.app.sensors;

import adas.app.AdasConfig;
import adas.app.Logger;
import adas.app.TimeUtil;
import adas.app.bridge.ProtoUtils;
import adas.app.bridge.ZMQBridgeService;
import adas.proto.CameraIntrinsicsOuterClass;
import adas.proto.Messages;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.graphics.Rect;
import android.hardware.camera2.params.MeteringRectangle;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Range;
import android.view.Surface;
import android.view.TextureView;
import android.view.WindowManager;
import android.graphics.SurfaceTexture;
import android.graphics.Matrix;
import android.graphics.RectF;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;

import android.graphics.Bitmap;
import adas.proto.Messages.ZMQMessage;
import adas.app.bridge.ProtoUtils;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class CameraHandler {

    private static final String TAG = "CameraHandler";

    public interface FailureListener {
        void onCameraFailed(String reason);
    }

    private final Context context;
    private HandlerThread backgroundThread;
    private Handler backgroundHandler;
    private ImageReader reader;
    private Surface previewSurface;
    private CaptureRequest.Builder captureRequest;
    private CameraCaptureSession captureSession;
    private CameraDevice cameraDevice;
    private CameraCharacteristics cameraCharacteristics;
    private FailureListener failureListener;
    private volatile boolean sessionAlive;

    TextureView preview;
    private int viewWidth;
    private int viewHeight;
    private int sensorOrientation = 90;
    private boolean lensFacingFront = false;

    /**
     * Swapped from another thread when the vision model changes in settings, so it is read exactly once
     * per frame below: two reads could disagree and hand the traffic branch the wrong buffer.
     */
    private volatile adas.app.vision.VisionPipeline visionPipeline;
    private adas.app.vision.TrafficVisionPipeline trafficVisionPipeline;
    private adas.app.vision.LaneOverlayView laneOverlay;

    public int W = 1280;
    public int H = 720;
    public int frameID = 0;

    private static final int JPEG_QUALITY = 70;
    private boolean chessboardCaptureMode;
    /** Chessboard mode: whether continuous AF has been frozen into a fixed focus distance. */
    private volatile boolean chessFocusLocked;
    private int chessAfSettledFrames;
    /** Whether `intrinsics_prior` was taken on this very phone. */
    private boolean ownPrior;
    private static volatile boolean recordCameraImages = true;

    /** Stopped cars produce nothing worth a JPEG. */
    private static final float BAG_MOVING_MPS = 0.5f;
    private static final long BAG_STOP_HOLD_MS = 3000;
    private static volatile float egoSpeedMps;
    private static volatile long egoSpeedAtMs;
    private static volatile long lastMovingMs;
    /** Logged once per transition so gaps in a bag are explainable rather than mysterious. */
    private static volatile boolean bagSkipping;

    /** Speed from `vehicle/state`, for the recording gate. Called from the ZMQ listener. */
    public static void setEgoSpeed(float mps) {
        egoSpeedMps = mps;
        egoSpeedAtMs = adas.app.TimeUtil.nowMs();
        if (mps > BAG_MOVING_MPS) {
            lastMovingMs = egoSpeedAtMs;
        }
    }

    /** Whether this frame is worth writing to the bag. */
    private static boolean bagWantsFrame() {
        final long now = adas.app.TimeUtil.nowMs();
        // No speed, or a stale one: record. Not knowing is not a reason to stop.
        if (egoSpeedAtMs == 0 || now - egoSpeedAtMs > 1000) {
            return true;
        }
        if (egoSpeedMps > BAG_MOVING_MPS) {
            return true;
        }
        return now - lastMovingMs <= BAG_STOP_HOLD_MS;
    }

    public static void setRecordCameraImages(boolean on) {
        recordCameraImages = on;
        Log.i("CameraHandler", "bag camera images: " + (on ? "on" : "off"));
    }

    public static boolean isRecordingCameraImages() {
        return recordCameraImages;
    }

    private static final int SCALE_FACTOR = 2;

    /** The two rates the switch offers. Anything else is refused rather than silently rounded. */
    public static final int FPS_MODEL = 20;
    public static final int FPS_FAST = 30;

    /**
     * Frames per second requested from the camera — and therefore the spacing of the pair the model sees, which is not a cosmetic choice.
     */
    private static volatile int targetFps = FPS_MODEL;

    /**
     * Pick the capture rate. Takes effect on the next {@link #start()} — the AE range belongs to the
     * capture session, so the session has to be rebuilt; the caller restarts the camera.
     */
    public static void setTargetFps(int fps) {
        targetFps = fps == FPS_FAST ? FPS_FAST : FPS_MODEL;
        Log.i("CameraHandler", "target fps -> " + targetFps + " (model step "
                + Math.round(1000f / targetFps) + " ms)");
    }

    public static int getTargetFps() {
        return targetFps;
    }

    private float bagFx;
    private float bagFy;
    private float bagCx;
    private float bagCy;
    private boolean bagIntrinsicsReady;
    /** Focal length in pixels for the full pipeline frame (W×H), as reported by the camera. */
    private volatile float measuredFocalPx;
    /** Where it came from: without this, "measured" and "defaulted" are indistinguishable. */
    private volatile String measuredFocalSource = "";
    /** Who to call once the focal length first becomes known. */
    private volatile Runnable onIntrinsicsReady;

    public CameraHandler(Context context, TextureView preview) {
        this.context = context;
        this.chessboardCaptureMode = AdasConfig.chessboardCapture(context);
        final String priorDevice = AdasConfig.intrinsicsPriorDevice(context);
        this.ownPrior = !priorDevice.isEmpty() && priorDevice.equalsIgnoreCase(android.os.Build.MODEL);
        Log.i(TAG, "intrinsics_prior was taken on '" + priorDevice + "', this is '" + android.os.Build.MODEL
                + "' — " + (ownPrior ? "ours, not sending factory values to the calibration service"
                                     : "foreign, sending factory values"));
        this.preview = preview;
        startBackgroundThread();
    }

    public void setVisionPipeline(adas.app.vision.VisionPipeline visionPipeline) {
        this.visionPipeline = visionPipeline;
    }

    public void setTrafficVisionPipeline(adas.app.vision.TrafficVisionPipeline trafficVisionPipeline) {
        this.trafficVisionPipeline = trafficVisionPipeline;
    }

    public void setLaneOverlay(adas.app.vision.LaneOverlayView laneOverlay) {
        this.laneOverlay = laneOverlay;
    }

    public void setFailureListener(FailureListener failureListener) {
        this.failureListener = failureListener;
    }

    /** Somebody else who wants the live frames — today the on-screen lens calibration. */
    public interface FrameTap {
        /** False to be skipped this frame; checked before the copy so a busy tap costs nothing. */
        boolean wantsFrame();

        void onFrame(adas.app.vision.YuvFrame yuv, long captureTsMs);
    }

    private volatile FrameTap frameTap;

    /** Pass null to detach. */
    public void setFrameTap(FrameTap tap) {
        this.frameTap = tap;
    }

    private void notifyFailed(String reason) {
        Log.e(TAG, "Camera failed: " + reason);
        sessionAlive = false;
        FailureListener l = failureListener;
        if (l != null) {
            l.onCameraFailed(reason);
        }
    }

    private void startBackgroundThread() {
        if (backgroundThread != null && backgroundThread.isAlive()) {
            return;
        }
        backgroundThread = new HandlerThread("CameraBackground");
        backgroundThread.start();
        backgroundHandler = new Handler(backgroundThread.getLooper());
    }

    private void stopBackgroundThread() {
        if (backgroundThread == null) {
            return;
        }
        backgroundThread.quitSafely();
        try {
            backgroundThread.join(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        backgroundThread = null;
        backgroundHandler = null;
    }

    public void configurePreviewTransform(int viewWidth, int viewHeight) {
        if (preview == null || viewWidth == 0 || viewHeight == 0) {
            return;
        }
        this.viewWidth = viewWidth;
        this.viewHeight = viewHeight;

        WindowManager wm = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
        int rotation = wm != null ? wm.getDefaultDisplay().getRotation() : Surface.ROTATION_0;

        Matrix matrix = new Matrix();
        RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);

        RectF bufferRect = new RectF(0, 0, H, W);
        float centerX = viewRect.centerX();
        float centerY = viewRect.centerY();

        if (rotation == Surface.ROTATION_90 || rotation == Surface.ROTATION_270) {
            bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
            float scale = Math.max((float) viewHeight / H, (float) viewWidth / W);
            matrix.postScale(scale, scale, centerX, centerY);
            matrix.postRotate(90f * (rotation - 2), centerX, centerY);
        } else if (rotation == Surface.ROTATION_180) {
            matrix.postRotate(180f, centerX, centerY);
        } else {

            float scale = Math.max((float) viewWidth / W, (float) viewHeight / H);
            float scaledW = W * scale;
            float scaledH = H * scale;
            float sx = scaledW / viewWidth;
            float sy = scaledH / viewHeight;
            float dx = (viewWidth - scaledW) * 0.5f;
            float dy = (viewHeight - scaledH) * 0.5f;
            matrix.setScale(sx, sy);
            matrix.postTranslate(dx, dy);
        }

        preview.setTransform(matrix);
        if (laneOverlay != null) {
            laneOverlay.setPreviewTransform(matrix, W, H, viewWidth, viewHeight);
        }

        int displayDeg = rotationToDegrees(rotation);
        int previewDeg = lensFacingFront
                ? (sensorOrientation + displayDeg) % 360
                : (sensorOrientation - displayDeg + 360) % 360;
        Log.i(TAG, String.format(
                "Preview transform view=%dx%d buf=%dx%d displayRot=%d sensorOri=%d previewDeg=%d",
                viewWidth, viewHeight, W, H, displayDeg, sensorOrientation, previewDeg));
    }

    private static int rotationToDegrees(int rotation) {
        switch (rotation) {
            case Surface.ROTATION_90: return 90;
            case Surface.ROTATION_180: return 180;
            case Surface.ROTATION_270: return 270;
            default: return 0;
        }
    }

    public void stop() {
        sessionAlive = false;
        if (captureSession != null) {
            try {
                captureSession.close();
            } catch (Exception e) {
                Log.w(TAG, "Error closing capture session", e);
            }
            captureSession = null;
        }
        if (cameraDevice != null) {
            try {
                cameraDevice.close();
            } catch (Exception e) {
                Log.w(TAG, "Error closing camera", e);
            }
            cameraDevice = null;
        }
        if (reader != null) {
            try {
                reader.close();
            } catch (Exception e) {
                Log.w(TAG, "Error closing ImageReader", e);
            }
            reader = null;
        }
        if (previewSurface != null) {
            try {
                previewSurface.release();
            } catch (Exception e) {
                Log.w(TAG, "Error releasing preview Surface", e);
            }
            previewSurface = null;
        }
        captureRequest = null;
    }

    public void release() {
        stop();
        stopBackgroundThread();
    }

    public boolean isCameraOpen() {
        return cameraDevice != null && sessionAlive;
    }

    public void start() {
        startBackgroundThread();
        if (cameraDevice != null || reader != null || captureSession != null) {
            stop();
        }
        android.hardware.camera2.CameraManager manager =
                (android.hardware.camera2.CameraManager) context.getSystemService(Context.CAMERA_SERVICE);

        if (manager == null) {
            notifyFailed("Unable to get camera manager");
            throw new RuntimeException("Unable to get camera manager.");
        }

        String cameraId = "0";

        try {
            cameraCharacteristics = manager.getCameraCharacteristics(cameraId);
            Integer so = cameraCharacteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
            if (so != null) {
                sensorOrientation = so;
            }
            Integer facing = cameraCharacteristics.get(CameraCharacteristics.LENS_FACING);
            lensFacingFront = facing != null && facing == CameraCharacteristics.LENS_FACING_FRONT;
            Log.i(TAG, "Camera characteristics: sensorOrientation=" + sensorOrientation
                    + " front=" + lensFacingFront);

            if (ActivityCompat.checkSelfPermission(context,
                    Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
                Log.e(TAG, "CAMERA permission not granted — openCamera skipped");
                return;
            }

            Log.i(TAG, "Opening camera id=" + cameraId);
            manager.openCamera(cameraId, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(@NonNull CameraDevice device) {
                    Log.i(TAG, "Camera onOpened");
                    cameraDevice = device;
                    if (viewWidth > 0 && viewHeight > 0) {
                        configurePreviewTransform(viewWidth, viewHeight);
                    } else if (preview != null) {
                        configurePreviewTransform(preview.getWidth(), preview.getHeight());
                    }
                    startCamera();
                }

                @Override
                public void onDisconnected(@NonNull CameraDevice device) {
                    Log.w(TAG, "Camera onDisconnected");
                    device.close();
                    cameraDevice = null;
                    sessionAlive = false;
                    closeCaptureResourcesOnly();
                    notifyFailed("Camera disconnected");
                }

                @Override
                public void onError(@NonNull CameraDevice device, int error) {
                    Log.e(TAG, "Camera onError: " + error);
                    device.close();
                    cameraDevice = null;
                    sessionAlive = false;
                    closeCaptureResourcesOnly();
                    notifyFailed("Camera error " + error);
                }
            }, backgroundHandler);
        } catch (CameraAccessException e) {
            Log.w(TAG, "Error getting camera configuration.", e);
            notifyFailed("CameraAccessException: " + e.getMessage());
        }
    }

    private void closeCaptureResourcesOnly() {
        if (captureSession != null) {
            try {
                captureSession.close();
            } catch (Exception ignored) {
            }
            captureSession = null;
        }
        if (reader != null) {
            try {
                reader.close();
            } catch (Exception ignored) {
            }
            reader = null;
        }
        if (previewSurface != null) {
            try {
                previewSurface.release();
            } catch (Exception ignored) {
            }
            previewSurface = null;
        }
        captureRequest = null;
    }

    private void startCamera() {
        List<Surface> list = new ArrayList<>();

        closeCaptureResourcesOnly();
        reader = ImageReader.newInstance(W, H, ImageFormat.YUV_420_888, 5);

        SurfaceTexture texture = preview.getSurfaceTexture();
        if (texture == null) {
            notifyFailed("Preview SurfaceTexture is null");
            return;
        }
        texture.setDefaultBufferSize(W, H);
        previewSurface = new Surface(texture);

        list.add(reader.getSurface());
        list.add(previewSurface);

        ImageReader.OnImageAvailableListener imageAvailableListener = new ImageReader.OnImageAvailableListener() {
            @Override
            public void onImageAvailable(ImageReader reader) {
                Image image = null;
                try {
                    image = reader.acquireLatestImage();
                    if (image == null) return;

                    final adas.app.vision.VisionPipeline vp = visionPipeline;
                    final FrameTap tap = frameTap;
                    final boolean tapWants = tap != null && tap.wantsFrame();
                    adas.app.vision.YuvFrame yuv = null;
                    if (vp != null || trafficVisionPipeline != null || tapWants) {
                        yuv = adas.app.vision.YuvFrame.copyFrom(image);
                    }

                    image.close();
                    image = null;

                    if (yuv != null) {
                        long captureTs = TimeUtil.nowMs();
                        if (vp != null) {
                            vp.submitYuv(yuv, captureTs);
                        }
                        if (trafficVisionPipeline != null) {
                            if (trafficVisionPipeline.wantsFrame()) {
                                adas.app.vision.YuvFrame trafficYuv =
                                        vp != null ? yuv.duplicate() : yuv;
                                trafficVisionPipeline.submitYuv(trafficYuv, captureTs);
                            }
                        }
                        if (tapWants) {
                            tap.onFrame(vp != null || trafficVisionPipeline != null ? yuv.duplicate() : yuv,
                                    captureTs);
                        }
                        if (recordCameraImages && Logger.getInstance().isRunning()) {
                            if (bagWantsFrame()) {
                                if (bagSkipping) {
                                    bagSkipping = false;
                                    Log.i(TAG, "moving again — camera frames back in the bag");
                                }
                                logBagJpegFromY(yuv, captureTs);
                            } else if (!bagSkipping) {
                                bagSkipping = true;
                                Log.i(TAG, String.format(java.util.Locale.US,
                                        "stopped (%.2f m/s) — camera frames paused after %d ms of hold",
                                        egoSpeedMps, BAG_STOP_HOLD_MS));
                            }
                        }
                    }
                } catch (Throwable t) {
                    Log.e(TAG, "frame processing failed", t);
                } finally {
                    if (image != null) {
                        try {
                            image.close();
                        } catch (Exception ignored) {
                        }
                    }
                }
            }
        };

        reader.setOnImageAvailableListener(imageAvailableListener, backgroundHandler);

        try {
            captureRequest = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            captureRequest.addTarget(list.get(0));
            captureRequest.addTarget(previewSurface);

            if (chessboardCaptureMode) {
                // Continuous AF only until it settles: the session then freezes the focus distance.
                // Refocusing between views changes the effective focal length (focus breathing), and a
                // single-fx solve over 30 views taken at different focus positions cannot fit — that is
                // how a flat, sharp board still comes back with a 5 px reprojection error.
                captureRequest.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                chessFocusLocked = false;
                chessAfSettledFrames = 0;
                Log.i(TAG, "Chessboard capture: AF on until settled, then locked for the whole session");
            } else {
                captureRequest.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            }

            Float minFocusDistance = cameraCharacteristics.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
            if (minFocusDistance != null && minFocusDistance > 0 && !chessboardCaptureMode) {

                captureRequest.set(CaptureRequest.LENS_FOCUS_DISTANCE, 0.0f);
                Log.i(TAG, "Set focus to infinity (0.0f) for calibration. Min focus distance: " + minFocusDistance);
            } else if (chessboardCaptureMode) {
                Log.i(TAG, "Focus: continuous AF (chessboard capture)");
            } else {
                Log.w(TAG, "Manual focus distance not supported on this device");
            }

            int[] availableOIS =
                    cameraCharacteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION);
            if (availableOIS != null && availableOIS.length > 0) {
                captureRequest.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                                 CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                Log.i(TAG, "Disabled optical image stabilization (OIS) for calibration");
            } else {
                Log.i(TAG, "Optical image stabilization (OIS) not available on this device");
            }

            int[] availableVideoStab =
                    cameraCharacteristics.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES);
            if (availableVideoStab != null && availableVideoStab.length > 0) {
                captureRequest.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                                 CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                Log.i(TAG, "Disabled video stabilization for calibration");
            } else {
                Log.i(TAG, "Video stabilization not available on this device");
            }

            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                int[] availableDistortionModes =
                        cameraCharacteristics.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
                if (availableDistortionModes != null && availableDistortionModes.length > 0) {
                    captureRequest.set(CaptureRequest.DISTORTION_CORRECTION_MODE,
                                     CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
                    Log.i(TAG, "Disabled lens distortion correction for calibration (raw distortion preserved)");
                } else {
                    Log.i(TAG, "Distortion correction modes not available on this device");
                }
            }

            // The model position wants a pinned interval; the fast one wants the highest rate.
            final Range<Integer> fps =
                    pickTargetFpsRange(cameraCharacteristics, targetFps, targetFps == FPS_MODEL);
            if (fps != null) {
                captureRequest.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps);
                Log.i(TAG, "Set target FPS range " + fps.getLower() + "-" + fps.getUpper());
            } else {
                Log.w(TAG, "No AE FPS ranges available; leaving default");
            }

            MeteringRectangle[] aeRegions = roadMeteringRegions(cameraCharacteristics, W, H);
            if (aeRegions != null) {
                captureRequest.set(CaptureRequest.CONTROL_AE_REGIONS, aeRegions);
            }

            logCameraIntrinsics();

        } catch (Exception e) {
            Log.e(TAG, "Error building capture request", e);
            notifyFailed("Capture request failed: " + e.getMessage());
            return;
        }

        try {
            List<OutputConfiguration> confs = new ArrayList<>();
            for (Surface surface : list) {
                confs.add(new OutputConfiguration(surface));
            }

            cameraDevice.createCaptureSession(
                    new SessionConfiguration(
                            SessionConfiguration.SESSION_REGULAR,
                            confs,
                            context.getMainExecutor(),
                            new CameraCaptureSession.StateCallback() {
                                @Override
                                public void onConfigured(CameraCaptureSession session) {
                                    captureSession = session;
                                    sessionAlive = true;
                                    startSession();
                                }

                                @Override
                                public void onConfigureFailed(CameraCaptureSession session) {
                                    sessionAlive = false;
                                    Log.e(TAG, "Capture session configuration failed");
                                    closeCaptureResourcesOnly();
                                    if (cameraDevice != null) {
                                        try {
                                            cameraDevice.close();
                                        } catch (Exception ignored) {
                                        }
                                        cameraDevice = null;
                                    }
                                    notifyFailed("Capture session configuration failed");
                                }
                            }
                    )
            );
        } catch (Throwable t) {
            Log.e(TAG, "createCaptureSession failed", t);
            notifyFailed("createCaptureSession: " + t.getMessage());
        }
    }

    /** Auto-exposure metering rectangle covering the road, so exposure is not computed against the sky. */
    private MeteringRectangle[] roadMeteringRegions(CameraCharacteristics chars, int frameWidth, int frameHeight) {
        Integer maxRegions = chars.get(CameraCharacteristics.CONTROL_MAX_REGIONS_AE);
        if (maxRegions == null || maxRegions < 1) {
            Log.w(TAG, "AE metering regions not supported on this device — exposure stays whole-frame");
            return null;
        }
        Rect array = chars.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE);
        if (array == null) {
            array = chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        }
        if (array == null || array.width() <= 0 || array.height() <= 0 || frameWidth <= 0 || frameHeight <= 0) {
            Log.w(TAG, "No active array size — cannot place the metering region");
            return null;
        }

        // The crop the sensor takes to produce our aspect ratio: one axis in full, the other centred.
        float wantAspect = (float) frameWidth / (float) frameHeight;
        float arrayAspect = (float) array.width() / (float) array.height();
        int cropW = array.width();
        int cropH = array.height();
        if (arrayAspect > wantAspect) {
            cropW = Math.round(array.height() * wantAspect);
        } else if (arrayAspect < wantAspect) {
            cropH = Math.round(array.width() / wantAspect);
        }
        int cropL = array.left + (array.width() - cropW) / 2;
        int cropT = array.top + (array.height() - cropH) / 2;

        int x = cropL + Math.round(cropW * 0.40f);
        int y = cropT + Math.round(cropH * 0.50f);
        int w = Math.max(1, Math.round(cropW * 0.20f));
        int h = Math.max(1, Math.round(cropH * 0.20f));
        Log.i(TAG, "AE metering on road: array " + array.width() + "x" + array.height()
                + ", crop " + cropW + "x" + cropH + " at " + cropL + "," + cropT
                + " -> region " + x + "," + y + " " + w + "x" + h + " (max regions " + maxRegions + ")");
        return new MeteringRectangle[]{new MeteringRectangle(x, y, w, h, MeteringRectangle.METERING_WEIGHT_MAX)};
    }

    /**
     * Picks the AE frame-rate range: highest achievable upper bound first, then the highest lower bound among equals — a high lower bound is what stops auto-exposure from dropping the rate for exposure.
     */
    private static Range<Integer> pickTargetFpsRange(CameraCharacteristics chars, int wanted) {
        return pickTargetFpsRange(chars, wanted, false);
    }

    /**
     * The two positions of the camera switch want opposite things.
     * @param pinExact prefer a range that pins one rate, even if it is not the fastest available.
     */
    private static Range<Integer> pickTargetFpsRange(CameraCharacteristics chars, int wanted,
                                                     boolean pinExact) {
        Range<Integer>[] ranges = chars.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        if (ranges == null || ranges.length == 0) {
            return null;
        }
        StringBuilder all = new StringBuilder();
        for (Range<Integer> r : ranges) {
            all.append(r.getLower()).append('-').append(r.getUpper()).append(' ');
        }
        Log.i(TAG, "AE fps ranges available: " + all.toString().trim());

        if (pinExact) {
            Range<Integer> nearest = null;
            for (Range<Integer> r : ranges) {
                if (!r.getLower().equals(r.getUpper())) {
                    continue;   // a range, not a rate: auto-exposure would wander inside it
                }
                if (nearest == null
                        || Math.abs(r.getUpper() - wanted) < Math.abs(nearest.getUpper() - wanted)
                        || (Math.abs(r.getUpper() - wanted) == Math.abs(nearest.getUpper() - wanted)
                            && r.getUpper() > nearest.getUpper())) {
                    nearest = r;
                }
            }
            if (nearest != null) {
                Log.i(TAG, "pinning " + nearest.getUpper() + " fps ("
                        + Math.round(1000f / nearest.getUpper()) + " ms step) — nearest fixed range to "
                        + wanted);
                return nearest;
            }
            Log.w(TAG, "no fixed AE range on this camera — falling back to the fastest under " + wanted);
        }

        Range<Integer> best = null;
        for (Range<Integer> r : ranges) {
            int upper = Math.min(r.getUpper(), wanted);
            if (upper <= 0) {
                continue;
            }
            // A range whose lower bound exceeds the target cannot deliver the target rate — it forces
            // its own. [60,60] ties [30,30] on the capped upper bound and would win on the lower one.
            if (r.getLower() > wanted) {
                continue;
            }
            if (best == null) {
                best = r;
                continue;
            }
            int bestUpper = Math.min(best.getUpper(), wanted);
            if (upper > bestUpper || (upper == bestUpper && r.getLower() > best.getLower())) {
                best = r;
            }
        }
        if (best == null) {
            return null;
        }
        if (!best.getLower().equals(best.getUpper())) {
            Log.w(TAG, "Best AE fps range " + best.getLower() + "-" + best.getUpper()
                    + " is not fixed — the sensor may still drop the rate in low light, but a fixed range at a"
                    + " lower rate would be worse");
        }
        return best;
    }

    private void startSession() {
        // Exposure and actual frame duration every ~5 s: if exposure falls and frame duration settles on
        // 1/fps after the metering region is set, the rate was being dropped by auto-exposure.
        CameraCaptureSession.CaptureCallback listener = new CameraCaptureSession.CaptureCallback() {
            private long lastLogNs = 0;

            public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                    TotalCaptureResult result) {
                super.onCaptureCompleted(session, request, result);

                if (chessboardCaptureMode && !chessFocusLocked) {
                    Integer afState = result.get(TotalCaptureResult.CONTROL_AF_STATE);
                    Float diopters = result.get(TotalCaptureResult.LENS_FOCUS_DISTANCE);
                    final boolean settled = afState != null
                            && (afState == TotalCaptureResult.CONTROL_AF_STATE_PASSIVE_FOCUSED
                                    || afState == TotalCaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED);
                    if (settled) {
                        // A handful of consecutive settled frames, so a single lucky one does not lock
                        // a focus the lens is still leaving.
                        if (++chessAfSettledFrames >= 8) {
                            chessFocusLocked = true;
                            if (diopters != null) {
                                try {
                                    captureRequest.set(CaptureRequest.CONTROL_AF_MODE,
                                            CaptureRequest.CONTROL_AF_MODE_OFF);
                                    captureRequest.set(CaptureRequest.LENS_FOCUS_DISTANCE, diopters);
                                    session.setRepeatingRequest(captureRequest.build(), this, backgroundHandler);
                                    Log.i(TAG, String.format("Chessboard: focus locked at %.2f dpt (~%.2f m)"
                                            + " — keep the board near this distance, vary only the tilt",
                                            diopters, diopters > 0.01f ? 1.0f / diopters : Float.POSITIVE_INFINITY));
                                } catch (Exception e) {
                                    Log.e(TAG, "Chessboard focus lock failed — AF stays continuous", e);
                                }
                            } else {
                                Log.w(TAG, "Chessboard: device reports no focus distance — AF stays"
                                        + " continuous; expect a higher reprojection error");
                            }
                        }
                    } else {
                        chessAfSettledFrames = 0;
                    }
                }

                Long exp = result.get(TotalCaptureResult.SENSOR_EXPOSURE_TIME);
                Long dur = result.get(TotalCaptureResult.SENSOR_FRAME_DURATION);
                Integer iso = result.get(TotalCaptureResult.SENSOR_SENSITIVITY);
                if (exp == null && dur == null) {
                    return;
                }
                long now = System.nanoTime();
                if (now - lastLogNs < 5_000_000_000L) {
                    return;
                }
                lastLogNs = now;
                Log.i(TAG, String.format("AE: exposure %.1f ms, frame duration %.1f ms (%.1f fps), iso %s",
                        exp == null ? Float.NaN : exp / 1e6f,
                        dur == null ? Float.NaN : dur / 1e6f,
                        dur == null || dur == 0 ? Float.NaN : 1e9f / dur,
                        iso == null ? "?" : iso.toString()));
            }
        };

        if (cameraDevice == null) return;

        try {
            captureSession.setRepeatingRequest(captureRequest.build(), listener, backgroundHandler);
        } catch (Exception e) {
            // The session may already be closed when the app restarts its handlers; the next
            // startCamera() opens a new one, so this is logged rather than escalated.
            Log.e(TAG, "setRepeatingRequest failed", e);
        }
    }

    /** Focal length in pixels for the W×H frame according to the camera itself, or 0 if it said nothing. */
    public float measuredFocalPx() {
        return measuredFocalPx;
    }

    public String measuredFocalSource() {
        return measuredFocalSource;
    }

    /** Call as soon as the focal length is known. If it already is, call immediately. */
    public void setOnIntrinsicsReady(Runnable listener) {
        onIntrinsicsReady = listener;
        if (listener != null && measuredFocalPx > 1f) {
            listener.run();
        }
    }

    private void logCameraIntrinsics() {
        try {
            StringBuilder intrinsicsData = new StringBuilder();
            intrinsicsData.append("=== CAMERA INTRINSIC PARAMETERS ===\n");

            float[] focalLengths = cameraCharacteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            if (focalLengths != null && focalLengths.length > 0) {
                intrinsicsData.append("Physical focal length: ").append(focalLengths[0]).append(" mm\n");
            }

            android.util.SizeF sensorSize = cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            if (sensorSize != null) {
                intrinsicsData.append("Sensor physical size: ").append(sensorSize.getWidth())
                              .append(" x ").append(sensorSize.getHeight()).append(" mm\n");
            }

            android.graphics.Rect activeArray =
                    cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
            if (activeArray != null) {
                intrinsicsData.append("Active pixel array: ").append(activeArray.width())
                              .append(" x ").append(activeArray.height()).append(" pixels\n");
            }

            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                float[] distortion = cameraCharacteristics.get(CameraCharacteristics.LENS_DISTORTION);
                if (distortion != null && distortion.length >= 5) {
                    intrinsicsData.append("Lens distortion coefficients: [")
                                  .append(distortion[0]).append(", ")
                                  .append(distortion[1]).append(", ")
                                  .append(distortion[2]).append(", ")
                                  .append(distortion[3]).append(", ")
                                  .append(distortion[4]).append("]\n");
                    intrinsicsData.append("Distortion model: k1, k2, k3, k4, k5 (radial + tangential)\n");
                }
            }

            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                float[] intrinsicCalibration =
                        cameraCharacteristics.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
                if (intrinsicCalibration != null && intrinsicCalibration.length >= 5) {
                    intrinsicsData.append("Lens intrinsic calibration: [")
                                  .append(intrinsicCalibration[0]).append(", ")
                                  .append(intrinsicCalibration[1]).append(", ")
                                  .append(intrinsicCalibration[2]).append(", ")
                                  .append(intrinsicCalibration[3]).append(", ")
                                  .append(intrinsicCalibration[4]).append("]\n");
                    intrinsicsData.append("Format: [fx, fy, cx, cy, s] where s=skew\n");
                }
            }

            if (focalLengths != null && focalLengths.length > 0 &&
                sensorSize != null && activeArray != null) {
                float focalLengthMm = focalLengths[0];
                float sensorWidthMm = sensorSize.getWidth();
                int imageWidthPx = activeArray.width();

                float focalLengthPx = (focalLengthMm / sensorWidthMm) * imageWidthPx;
                intrinsicsData.append("Calculated focal length in pixels (fx): ").append(focalLengthPx).append(" px\n");
                intrinsicsData.append("Note: This is approximate. Use calibration for accurate values.\n");
            }

            intrinsicsData.append("Capture resolution: ").append(W).append(" x ").append(H).append(" pixels\n");
            intrinsicsData.append("=== End of camera intrinsics ===");

            try {
                long currentTime = TimeUtil.nowMs();

                float physicalFocalLengthMm = 0.0f;
                float sensorWidthMm = 0.0f, sensorHeightMm = 0.0f;
                int activeArrayWidth = 0, activeArrayHeight = 0;
                float[] distortionCoefficients = null;
                float[] intrinsicCalibration = null;
                float focalLengthPxFull = 0.0f;
                String distortionModel = "radial_tangential";

                if (focalLengths != null && focalLengths.length > 0) {
                    physicalFocalLengthMm = focalLengths[0];
                }
                if (sensorSize != null) {
                    sensorWidthMm = sensorSize.getWidth();
                    sensorHeightMm = sensorSize.getHeight();
                }
                if (activeArray != null) {
                    activeArrayWidth = activeArray.width();
                    activeArrayHeight = activeArray.height();
                }
                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                    distortionCoefficients = cameraCharacteristics.get(CameraCharacteristics.LENS_DISTORTION);
                    intrinsicCalibration = cameraCharacteristics.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
                }

                // Which of the three the numbers came from travels with them: the fields look the same either way, and a bag that cannot tell a measured lens from a fixed default cannot be trusted about distances.
                CameraIntrinsicsOuterClass.CameraIntrinsics.Source intrinsicsSource;
                final boolean lensCalibUsable = intrinsicCalibration != null && intrinsicCalibration.length >= 4
                        && intrinsicCalibration[0] > 1f && intrinsicCalibration[1] > 1f && activeArrayWidth > 0;
                if (lensCalibUsable) {
                    float sx = (float) W / activeArrayWidth;
                    focalLengthPxFull = intrinsicCalibration[0] * sx;
                    intrinsicsSource = CameraIntrinsicsOuterClass.CameraIntrinsics.Source.SOURCE_LENS_CALIBRATION;
                } else if (focalLengths != null && focalLengths.length > 0 && sensorWidthMm > 0) {
                    focalLengthPxFull = (physicalFocalLengthMm / sensorWidthMm) * W;
                    intrinsicsSource = CameraIntrinsicsOuterClass.CameraIntrinsics.Source.SOURCE_FOCAL_ESTIMATE;
                } else {
                    focalLengthPxFull = 930f;
                    intrinsicsSource = CameraIntrinsicsOuterClass.CameraIntrinsics.Source.SOURCE_FALLBACK;
                }
                if (lensCalibUsable && focalLengths != null && focalLengths.length > 0 && sensorWidthMm > 0) {
                    // Both values into the log: the gap between them is the measure of how far the
                    // datasheet figures can be trusted on this phone.
                    Log.i(TAG, String.format(java.util.Locale.US,
                            "focal length: lens calibration %.1f px, datasheet estimate %.1f px — taking the calibration",
                            focalLengthPxFull, (physicalFocalLengthMm / sensorWidthMm) * W));
                }

                // The bag frame is the same frame, only half the size, so the numbers are the same divided by two.
                if (intrinsicCalibration != null && intrinsicCalibration.length >= 4 && !lensCalibUsable) {
                    Log.w(TAG, "LENS_INTRINSIC_CALIBRATION present but unusable "
                            + "(zeros, or the sensor size is unknown) — using the datasheet estimate");
                }
                int bagW = W / SCALE_FACTOR;
                int bagH = H / SCALE_FACTOR;
                float scale = 1.0f / SCALE_FACTOR;
                float fxBag = focalLengthPxFull * scale;
                float fyBag = focalLengthPxFull * scale;
                // The principal point is the frame centre. It cannot be taken from the lens
                // calibration without recomputing the vertical 16:9 crop of the 4:3 sensor, and doing
                // so buys no accuracy.
                float cxBag = bagW * 0.5f;
                float cyBag = bagH * 0.5f;

                // What we hand out is the focal length for the full frame: the model warp works with
                // W×H, not with the downscaled bag frame.
                final boolean firstTime = measuredFocalPx <= 1f;
                measuredFocalPx = focalLengthPxFull;
                measuredFocalSource = intrinsicsSource.name();
                if (firstTime && measuredFocalPx > 1f) {
                    final Runnable listener = onIntrinsicsReady;
                    if (listener != null) {
                        listener.run();
                    }
                }

                bagFx = fxBag;
                bagFy = fyBag;
                bagCx = cxBag;
                bagCy = cyBag;
                bagIntrinsicsReady = true;

                {
                    // In units of the FULL pipeline frame, not the downscaled bag frame.
                    float[] frameK = new float[]{fxBag / scale, fyBag / scale, cxBag / scale, cyBag / scale, 0f};
                    Messages.ZMQMessage intrinsicsMessage = ProtoUtils.createCameraIntrinsicsMessage(
                        physicalFocalLengthMm,
                        sensorWidthMm, sensorHeightMm,
                        activeArrayWidth, activeArrayHeight,
                        distortionCoefficients,
                        frameK,
                        focalLengthPxFull,
                        W, H,
                        "camera_0",
                        distortionModel,
                        intrinsicsSource,
                        currentTime);
                    // Both ways on purpose. The bag needs it to make an offline replay reproducible,
                    // and the calibration service needs it to stop running on the config's prior —
                    // logging alone never left the phone's storage.
                    Logger.getInstance().logZMQMessage(intrinsicsMessage);
                    // To the calibration service only when the prior was NOT taken on this phone.
                    // Our own prior comes from a chessboard and is more accurate than the factory
                    // characteristics: on the OnePlus it is 993.4 against 951. Overriding it with the
                    // factory value would degrade something measured.
                    if (!ownPrior) {
                        ZMQBridgeService.publishToNative(intrinsicsMessage);
                    }
                    Log.i(TAG, String.format("Bag frame intrinsics (JPEG %dx%d): fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                            bagW, bagH, fxBag, fyBag, cxBag, cyBag));
                }
            } catch (Exception e) {
                Log.e(TAG, "Error creating camera intrinsics protobuf message for bag logging", e);
            }

        } catch (Exception e) {
            String errorMsg = "Error logging camera intrinsics: " + e.getMessage();
            Log.e(TAG, errorMsg, e);
        }
    }

    public void ensureBagIntrinsicsLogged() {
        if (cameraCharacteristics != null) {
            logCameraIntrinsics();
        }
    }

    private void logBagJpegFromY(adas.app.vision.YuvFrame yuv, long timestampMs) {
        try {
            int bagW = W / SCALE_FACTOR;
            int bagH = H / SCALE_FACTOR;
            int[] pixels = new int[bagW * bagH];
            for (int j = 0; j < bagH; j++) {
                int srcRow = (j * SCALE_FACTOR) * yuv.width;
                int dstRow = j * bagW;
                for (int i = 0; i < bagW; i++) {
                    int y = yuv.y[srcRow + i * SCALE_FACTOR] & 0xff;
                    pixels[dstRow + i] = 0xff000000 | (y << 16) | (y << 8) | y;
                }
            }
            Bitmap bitmap = Bitmap.createBitmap(pixels, bagW, bagH, Bitmap.Config.ARGB_8888);
            java.io.ByteArrayOutputStream stream = new java.io.ByteArrayOutputStream();
            bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, stream);
            bitmap.recycle();
            byte[] imageData = stream.toByteArray();

            float fx = bagIntrinsicsReady ? bagFx : 930f / SCALE_FACTOR;
            float fy = bagIntrinsicsReady ? bagFy : 930f / SCALE_FACTOR;
            float cx = bagIntrinsicsReady ? bagCx : bagW * 0.5f;
            float cy = bagIntrinsicsReady ? bagCy : bagH * 0.5f;

            ZMQMessage cameraMessage = ProtoUtils.createCameraImageMessage(
                    imageData, bagW, bagH, "JPEG", frameID++, timestampMs, fx, fy, cx, cy);
            Logger.getInstance().logZMQMessage(cameraMessage);
        } catch (Exception e) {
            Log.e(TAG, "bag JPEG from Y failed", e);
        }
    }
}
