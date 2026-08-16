package adas.app.vision;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Camera intrinsics from a printed chessboard, measured on the phone that will drive.
 *
 * <p>The running system takes fx/fy/cx/cy from {@code intrinsics_prior} in the config — a number typed
 * in for whichever phone it was typed in for. The device's own {@code LENS_INTRINSIC_CALIBRATION} is
 * better where it exists, but plenty of devices leave it empty. A chessboard measures the lens
 * actually fitted, and it is the only one of the three that also yields distortion.
 *
 * <p>Follows flowpilot's {@code CameraCalibratorIntrinsic}: find the corners, refine them to sub-pixel,
 * and keep the view only when it differs enough from the ones already held. That last part is what
 * makes it work — thirty photographs of the same pose constrain the focal length no better than one,
 * and the solver will happily return a confident wrong answer from them.
 *
 * <p>No UI and no camera here: frames are pushed in from whoever owns the stream, so this runs against
 * the pipeline's live camera instead of opening a second session that Android would refuse.
 */
public final class IntrinsicsCalibrator {
    private static final String TAG = "IntrinsicsCalib";

    /** Inner corners, not squares — this is what {@code findChessboardCorners} counts. */
    public static final int PATTERN_COLS = 9;
    public static final int PATTERN_ROWS = 6;

    /** Views to collect before solving. Below about twenty the focal length stays loose. */
    public static final int TARGET_VIEWS = 30;

    /**
     * Least mean corner displacement against every kept view [px] for a new one to count. Holding the
     * board still would otherwise fill the quota with copies of one pose.
     */
    private static final double MIN_NOVELTY_PX = 50.0;

    /** Above this the board was blurred, bent or half out of frame; the fit is not worth keeping. */
    private static final double MAX_REPROJECTION_PX = 1.0;

    /** What the last frame produced, for the overlay to draw. */
    public static final class Detection {
        public final float[] corners;  ///< Flat x,y pairs in image pixels; null when nothing was found.
        public final boolean accepted; ///< True when the view was novel enough to keep.

        Detection(float[] corners, boolean accepted) {
            this.corners = corners;
            this.accepted = accepted;
        }
    }

    /** A solved calibration. */
    public static final class Result {
        public final double fx, fy, cx, cy;
        public final double reprojectionPx;
        public final int width, height;
        public final boolean ok;
        public final String message;

        Result(boolean ok, double fx, double fy, double cx, double cy, double reprojectionPx,
               int width, int height, String message) {
            this.ok = ok;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.reprojectionPx = reprojectionPx;
            this.width = width;
            this.height = height;
            this.message = message;
        }
    }

    private final List<Mat> kept = new ArrayList<>();
    private int width;
    private int height;

    /** How many distinct views are held. */
    public int keptViews() {
        return kept.size();
    }

    public boolean isFull() {
        return kept.size() >= TARGET_VIEWS;
    }

    public void clear() {
        for (Mat m : kept) {
            m.release();
        }
        kept.clear();
        width = 0;
        height = 0;
    }

    /**
     * Offer one frame.
     *
     * @param y      Luminance plane, tightly packed, {@code w * h} bytes. Corner detection is a
     *               grayscale problem, so the chroma planes are never touched.
     * @param w      Frame width [px].
     * @param h      Frame height [px].
     * @return What was found, for drawing; never null.
     */
    public Detection offer(byte[] y, int w, int h) {
        if (isFull()) {
            return new Detection(null, false);
        }
        if (width == 0) {
            width = w;
            height = h;
        } else if (width != w || height != h) {
            // Mixing sizes would mix pixel scales in one solve; the answer would be neither.
            Log.w(TAG, "frame size changed mid-collection — ignoring");
            return new Detection(null, false);
        }

        Mat gray = new Mat(h, w, CvType.CV_8UC1);
        gray.put(0, 0, y);
        MatOfPoint2f corners = new MatOfPoint2f();
        boolean found;
        try {
            found = Calib3d.findChessboardCorners(gray, new Size(PATTERN_COLS, PATTERN_ROWS), corners,
                    Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE
                            | Calib3d.CALIB_CB_FAST_CHECK);
            if (found) {
                Imgproc.cornerSubPix(gray, corners, new Size(7, 7), new Size(-1, -1),
                        new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 40, 0.001));
            }
        } finally {
            gray.release();
        }

        if (!found) {
            corners.release();
            return new Detection(null, false);
        }

        final float[] flat = flatten(corners);
        final boolean novel = isNovel(corners);
        if (novel) {
            kept.add(corners);
        } else {
            corners.release();
        }
        return new Detection(flat, novel);
    }

    /**
     * Corners as a flat x,y array.
     *
     * <p>Through {@code toArray} rather than {@code Mat.get}: the matrix is {@code CV_32FC2}, and the
     * double-buffer overload of {@code get} rejects it outright — the detector found the board and the
     * read threw, which looked from outside like a board that was never seen.
     */
    private static float[] flatten(MatOfPoint2f corners) {
        final Point[] points = corners.toArray();
        final float[] flat = new float[points.length * 2];
        for (int i = 0; i < points.length; i++) {
            flat[2 * i] = (float) points[i].x;
            flat[2 * i + 1] = (float) points[i].y;
        }
        return flat;
    }

    /**
     * True when this view is far enough from every view already kept.
     *
     * <p>The corners come back in the same order every time, so comparing point by point is
     * meaningful: the mean displacement is how far the board moved in the image between the two.
     */
    private boolean isNovel(MatOfPoint2f candidate) {
        final Point[] a = candidate.toArray();
        for (Mat prev : kept) {
            final Point[] b = new MatOfPoint2f(prev).toArray();
            if (b.length != a.length) {
                continue;
            }
            double sum = 0.0;
            for (int i = 0; i < a.length; i++) {
                sum += Math.hypot(a[i].x - b[i].x, a[i].y - b[i].y);
            }
            if (sum / a.length < MIN_NOVELTY_PX) {
                return false;
            }
        }
        return true;
    }

    /**
     * Solve for the camera matrix from the views collected so far.
     *
     * <p>The collected views are released either way: a solve consumes them, and a failed solve is not
     * improved by keeping the frames that failed it.
     */
    public Result solve() {
        if (kept.isEmpty()) {
            return new Result(false, 0, 0, 0, 0, 0, width, height, "no views collected");
        }

        List<Mat> objectPoints = new ArrayList<>(kept.size());
        for (int i = 0; i < kept.size(); i++) {
            objectPoints.add(boardPoints());
        }
        Mat cameraMatrix = new Mat();
        Mat distortion = new Mat();
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        try {
            double rms = Calib3d.calibrateCamera(objectPoints, kept, new Size(width, height),
                    cameraMatrix, distortion, rvecs, tvecs);
            double fx = cameraMatrix.get(0, 0)[0];
            double fy = cameraMatrix.get(1, 1)[0];
            double cx = cameraMatrix.get(0, 2)[0];
            double cy = cameraMatrix.get(1, 2)[0];
            Log.i(TAG, String.format(Locale.US, "fx=%.1f fy=%.1f cx=%.1f cy=%.1f rms=%.3f px from %d views",
                    fx, fy, cx, cy, rms, kept.size()));

            if (rms > MAX_REPROJECTION_PX || !(fx > 1.0) || !(fy > 1.0)) {
                return new Result(false, fx, fy, cx, cy, rms, width, height,
                        String.format(Locale.US, "reprojection %.2f px — flatten the board and retry", rms));
            }
            return new Result(true, fx, fy, cx, cy, rms, width, height, "ok");
        } catch (Exception e) {
            Log.e(TAG, "calibrateCamera failed", e);
            return new Result(false, 0, 0, 0, 0, 0, width, height, String.valueOf(e.getMessage()));
        } finally {
            for (Mat m : objectPoints) {
                m.release();
            }
            for (Mat m : rvecs) {
                m.release();
            }
            for (Mat m : tvecs) {
                m.release();
            }
            cameraMatrix.release();
            distortion.release();
            clear();
        }
    }

    /** The board in its own coordinates, one unit per square: only ratios reach the intrinsics. */
    private static MatOfPoint3f boardPoints() {
        List<Point3> points = new ArrayList<>(PATTERN_COLS * PATTERN_ROWS);
        for (int row = 0; row < PATTERN_ROWS; row++) {
            for (int col = 0; col < PATTERN_COLS; col++) {
                points.add(new Point3(col, row, 0.0));
            }
        }
        MatOfPoint3f mat = new MatOfPoint3f();
        mat.fromList(points);
        return mat;
    }
}
