package ai.flow.adas;

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
import ai.flow.adas.Messages.ZMQMessage;
import ai.flow.adas.ProtoUtils;

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
    private volatile ai.flow.adas.vision.VisionPipeline visionPipeline;
    private ai.flow.adas.vision.TrafficVisionPipeline trafficVisionPipeline;
    private ai.flow.adas.vision.LaneOverlayView laneOverlay;

    public int W = 1280;
    public int H = 720;
    public int frameID = 0;

    private static final int JPEG_QUALITY = 70;
    private boolean chessboardCaptureMode;
    private static volatile boolean recordCameraImages = true;

    public static void setRecordCameraImages(boolean on) {
        recordCameraImages = on;
        Log.i("CameraHandler", "bag camera images: " + (on ? "on" : "off"));
    }

    public static boolean isRecordingCameraImages() {
        return recordCameraImages;
    }

    private static final int SCALE_FACTOR = 2;
    /**
     * Кадров в секунду, которых просим у камеры. Было 20, устройство отдавало 22.7 (то есть точного
     * диапазона (20,20) у него нет и камера бежит свободно).
     *
     * Почему это важно для темпа зрения: конвейер обрабатывает кадр за 59 мс медианой (подготовка 9 +
     * инференс 45.5), а период камеры был 44 мс. Закончив счёт, конвейер обнаруживает, что следующий
     * кадр придёт только через 29 мс, и темп квантуется по периоду камеры — ровно половина камерного,
     * 11.3 Гц (замерено: 2.02 кадра камеры на один обработанный, бег 2026_08_04_21_00_18). При 30 к/с
     * квант становится 33 мс и ожидание падает до ~7 мс.
     *
     * Потолок при любой частоте камеры — время работы конвейера, ~15 Гц; 20 Гц апстрима требуют
     * уложить подготовку с инференсом в один период камеры.
     */
    private static final int TARGET_FPS = 30;

    private float bagFx;
    private float bagFy;
    private float bagCx;
    private float bagCy;
    private boolean bagIntrinsicsReady;

    public CameraHandler(Context context, TextureView preview) {
        this.context = context;
        this.chessboardCaptureMode = AdasConfig.chessboardCapture(context);
        this.preview = preview;
        startBackgroundThread();
    }

    public void setVisionPipeline(ai.flow.adas.vision.VisionPipeline visionPipeline) {
        this.visionPipeline = visionPipeline;
    }

    public void setTrafficVisionPipeline(ai.flow.adas.vision.TrafficVisionPipeline trafficVisionPipeline) {
        this.trafficVisionPipeline = trafficVisionPipeline;
    }

    public void setLaneOverlay(ai.flow.adas.vision.LaneOverlayView laneOverlay) {
        this.laneOverlay = laneOverlay;
    }

    public void setFailureListener(FailureListener failureListener) {
        this.failureListener = failureListener;
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
        android.hardware.camera2.CameraManager manager = (android.hardware.camera2.CameraManager) context.getSystemService(Context.CAMERA_SERVICE);

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

            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
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

                    final ai.flow.adas.vision.VisionPipeline vp = visionPipeline;
                    ai.flow.adas.vision.YuvFrame yuv = null;
                    if (vp != null || trafficVisionPipeline != null) {
                        yuv = ai.flow.adas.vision.YuvFrame.copyFrom(image);
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
                                ai.flow.adas.vision.YuvFrame trafficYuv =
                                        vp != null ? yuv.duplicate() : yuv;
                                trafficVisionPipeline.submitYuv(trafficYuv, captureTs);
                            }
                        }
                        if (recordCameraImages && Logger.getInstance().isRunning()) {
                            logBagJpegFromY(yuv, captureTs);
                        }
                    }
                } catch (Throwable t) {
                    t.printStackTrace();
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
                captureRequest.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                Log.w(TAG, "Chessboard capture: autofocus enabled");
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

            int[] availableOIS = cameraCharacteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION);
            if (availableOIS != null && availableOIS.length > 0) {
                captureRequest.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                                 CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);
                Log.i(TAG, "Disabled optical image stabilization (OIS) for calibration");
            } else {
                Log.i(TAG, "Optical image stabilization (OIS) not available on this device");
            }

            int[] availableVideoStab = cameraCharacteristics.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES);
            if (availableVideoStab != null && availableVideoStab.length > 0) {
                captureRequest.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                                 CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
                Log.i(TAG, "Disabled video stabilization for calibration");
            } else {
                Log.i(TAG, "Video stabilization not available on this device");
            }

            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                int[] availableDistortionModes = cameraCharacteristics.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES);
                if (availableDistortionModes != null && availableDistortionModes.length > 0) {
                    captureRequest.set(CaptureRequest.DISTORTION_CORRECTION_MODE,
                                     CaptureRequest.DISTORTION_CORRECTION_MODE_OFF);
                    Log.i(TAG, "Disabled lens distortion correction for calibration (raw distortion preserved)");
                } else {
                    Log.i(TAG, "Distortion correction modes not available on this device");
                }
            }

            Range<Integer> fps = pickTargetFpsRange(cameraCharacteristics, TARGET_FPS);
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

    /**
     * Прямоугольник замера экспозиции, накрывающий дорогу.
     *
     * Единственная настройка камеры, которая есть у flowpilot и которой не было у нас: экспозиция считалась
     * по всему кадру вместе с небом, дорога систематически недоэкспонирована, и AE удлиняла выдержку — а с
     * плавающим диапазоном частоты это роняло камеру (см. {@link #pickTargetFpsRange}).
     *
     * Порт НЕ дословный, и в этом суть. flowpilot считает прямоугольник от размера кадра
     * (`W = Camera.frameSize[0]`, 1280x720), а `CONTROL_AE_REGIONS` задаётся в координатах активной области
     * сенсора. На сенсоре 4000x3000 их прямоугольник попал бы в левый верхний угол, то есть ровно в небо —
     * туда, откуда мы экспозицию и уводим. Здесь отображение делается честно: сначала находится вырез, из
     * которого сенсор формирует наш кадр с его соотношением сторон, затем внутри выреза берётся та же доля,
     * что у них — по горизонтали 0.4..0.6, по вертикали 0.5..0.7, то есть центр чуть ниже горизонта.
     *
     * Система координат зависит от коррекции дисторсии: при DISTORTION_CORRECTION_MODE_OFF это
     * pre-correction активная область, иначе обычная. Мы дисторсию выключаем, поэтому предпочитается
     * pre-correction.
     */
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

        // Вырез, из которого сенсор делает кадр нашего соотношения сторон: по одной оси берётся всё, по
        // другой — центрированная часть. Без этого шага прямоугольник уезжает вверх на всю разницу форматов.
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
     * Диапазон частоты кадров для AE.
     *
     * Здесь было две ошибки подряд, и вторая хуже первой.
     *
     * Сначала брался любой диапазон, ПОКРЫВАЮЩИЙ цель: при отсутствии [30,30] подходил [15,30], а нижняя
     * граница 15 разрешает автоэкспозиции ронять частоту вдвое ради выдержки. На вечернем заезде
     * 2026_08_07_19_04_05 это дало интервал 67 мс (14.9 Гц) при работе конвейера 52 мс.
     *
     * Тогда стал предпочитаться ФИКСИРОВАННЫЙ диапазон — и это оказалось хуже. На заезде
     * 2026_08_08_10_47_41 интервал встал ровно на 100 мс с модой на 100: единственный фиксированный
     * диапазон у этого телефона оказался низким, и правило прибило камеру к 10 fps. Темп упал 13.5 -> 10.06
     * Гц, а шаг уставки на дугах вырос 0.49 -> 0.91 градуса. Фиксированность сама по себе бесполезна, если
     * фиксирует на низкой частоте.
     *
     * Правильный порядок предпочтений: сначала максимальная ДОСТИЖИМАЯ частота (верхняя граница), и уже
     * среди равных по верхней — максимальная нижняя, потому что высокая нижняя граница и есть то, что
     * мешает AE ронять частоту. То есть [30,30] лучше [15,30], а [15,30] лучше [10,10] — последнее правило
     * и было нарушено.
     */
    private static Range<Integer> pickTargetFpsRange(CameraCharacteristics chars, int targetFps) {
        Range<Integer>[] ranges = chars.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
        if (ranges == null || ranges.length == 0) {
            return null;
        }
        StringBuilder all = new StringBuilder();
        for (Range<Integer> r : ranges) {
            all.append(r.getLower()).append('-').append(r.getUpper()).append(' ');
        }
        Log.i(TAG, "AE fps ranges available: " + all.toString().trim());

        Range<Integer> best = null;
        for (Range<Integer> r : ranges) {
            int upper = Math.min(r.getUpper(), targetFps);
            if (upper <= 0) {
                continue;
            }
            // Диапазон, чья НИЖНЯЯ граница выше цели, целевой темп выдать не может — он навяжет
            // свой. Например [60,60] на телефоне с 60-герцовым превью: по верхней границе он даёт
            // ничью с [30,30], а по нижней выигрывает, и камера уходит на 60 Гц. Это удвоило бы
            // работу потока камеры ради конвейера, который столько не съест, и вдвое урезало бы
            // выдержку — ровно та беда со светом, от которой это правило и написано.
            if (r.getLower() > targetFps) {
                continue;
            }
            if (best == null) {
                best = r;
                continue;
            }
            int bestUpper = Math.min(best.getUpper(), targetFps);
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
        // Экспозиция и реальная длительность кадра раз в ~5 с. Это то, чем доказывается механизм: если после
        // области замера выдержка падает, а длительность кадра встаёт на 1/fps — значит камеру уронила
        // именно автоэкспозиция по небу, а не что-то другое. Без этих строк вечерний заезд снова оставил бы
        // только «камера отдаёт 15 Гц» без причины.
        CameraCaptureSession.CaptureCallback listener = new CameraCaptureSession.CaptureCallback() {
            private long lastLogNs = 0;

            public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result) {
                super.onCaptureCompleted(session, request, result);
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
            e.printStackTrace();
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

            android.graphics.Rect activeArray = cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
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
                float[] intrinsicCalibration = cameraCharacteristics.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
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

                if (focalLengths != null && focalLengths.length > 0 && sensorWidthMm > 0) {
                    focalLengthPxFull = (physicalFocalLengthMm / sensorWidthMm) * W;
                } else if (intrinsicCalibration != null && intrinsicCalibration.length >= 4) {
                    float sx = (float) W / Math.max(1, activeArrayWidth);
                    focalLengthPxFull = intrinsicCalibration[0] * sx;
                } else {
                    focalLengthPxFull = 930f;
                }

                int bagW = W / SCALE_FACTOR;
                int bagH = H / SCALE_FACTOR;
                float scale = 1.0f / SCALE_FACTOR;
                float fxBag = focalLengthPxFull * scale;
                float fyBag = focalLengthPxFull * scale;
                float cxBag = bagW * 0.5f;
                float cyBag = bagH * 0.5f;
                if (intrinsicCalibration != null && intrinsicCalibration.length >= 4
                        && intrinsicCalibration[0] > 1f && intrinsicCalibration[1] > 1f) {
                    float sx = (float) bagW / Math.max(1, activeArrayWidth);
                    float sy = (float) bagH / Math.max(1, activeArrayHeight);
                    fxBag = intrinsicCalibration[0] * sx;
                    fyBag = intrinsicCalibration[1] * sy;
                    cxBag = intrinsicCalibration[2] * sx;
                    cyBag = intrinsicCalibration[3] * sy;
                } else if (intrinsicCalibration != null && intrinsicCalibration.length >= 4) {
                    Log.w(TAG, "LENS_INTRINSIC_CALIBRATION present but zero — using focal-length estimate");
                }

                bagFx = fxBag;
                bagFy = fyBag;
                bagCx = cxBag;
                bagCy = cyBag;
                bagIntrinsicsReady = true;

                if (Logger.getInstance().isRunning()) {
                    float[] bagK = new float[]{fxBag, fyBag, cxBag, cyBag, 0f};
                    Messages.ZMQMessage intrinsicsMessage = ProtoUtils.createCameraIntrinsicsMessage(
                        physicalFocalLengthMm,
                        sensorWidthMm, sensorHeightMm,
                        activeArrayWidth, activeArrayHeight,
                        distortionCoefficients,
                        bagK,
                        fxBag,
                        bagW, bagH,
                        "camera_0",
                        distortionModel,
                        currentTime);
                    Logger.getInstance().logZMQMessage(intrinsicsMessage);
                    Log.i(TAG, String.format("Bag intrinsics (JPEG %dx%d): fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
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

    private void logBagJpegFromY(ai.flow.adas.vision.YuvFrame yuv, long timestampMs) {
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
