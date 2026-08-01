package ai.flow.adas.vision;

import android.graphics.Bitmap;

/**
 * Flowpilot-style model warp: camera K + calib RPY → 3×3 homography that maps
 * model-frame pixels → camera pixels (same convention as TransformCL / OpenCL).
 *
 * {@code warp = K · view_from_device · R(rpy) · inv(medmodel_K · view_from_device)}
 */
public final class ModelCalibWarp {
    public static final int MODEL_W = 512;
    public static final int MODEL_H = 256;

    /** openpilot / flowpilot medmodel intrinsics. */
    private static final float MED_FL = 910.0f;
    private static final float MED_CY = 47.6f;

    /** view_from_device: device (x forward, y right, z down) → camera view. */
    private static final float[] VIEW_FROM_DEVICE = {
            0, 1, 0,
            0, 0, 1,
            1, 0, 0
    };

    private static volatile Boolean nativeWarpOk;

    private ModelCalibWarp() {}

    private static boolean ensureNativeWarp() {
        Boolean ok = nativeWarpOk;
        if (ok != null) {
            return ok;
        }
        synchronized (ModelCalibWarp.class) {
            if (nativeWarpOk != null) {
                return nativeWarpOk;
            }
            try {
                System.loadLibrary("adas_app_android");
                nativeWarpOk = true;
            } catch (UnsatisfiedLinkError e) {
                nativeWarpOk = false;
            }
            return nativeWarpOk;
        }
    }

    /**
     * Native TransformCL-equivalent warp. Implemented in libadas_app_android.
     * @return true on success
     */
    private static native boolean nativeWarpYuvToFrame6(
            byte[] y, byte[] u, byte[] v, int width, int height, float[] mModelToCam, float[] out6);

    /**
     * @param rollRad  calib roll (rad)
     * @param pitchRad calib pitch (rad)
     * @param yawRad   calib yaw (rad)
     * @param fx,fy,cx,cy camera intrinsics for the source bitmap size
     * @return row-major 3×3, model → camera
     */
    public static float[] warpMatrix(double rollRad, double pitchRad, double yawRad,
                                     float fx, float fy, float cx, float cy) {
        float[] K = {
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1
        };
        float[] medK = {
                MED_FL, 0, 0.5f * MODEL_W,
                0, MED_FL, MED_CY,
                0, 0, 1
        };
        float[] medFromCalib = mul3(medK, VIEW_FROM_DEVICE);
        float[] calibFromModel = inv3(medFromCalib);
        float[] deviceFromCalib = rotFromEuler(rollRad, pitchRad, yawRad);
        float[] viewFromCalib = mul3(VIEW_FROM_DEVICE, deviceFromCalib);
        float[] cameraFromCalib = mul3(K, viewFromCalib);
        return mul3(cameraFromCalib, calibFromModel);
    }

    /** Same as {@link #warpMatrix} with degrees. */
    public static float[] warpMatrixDeg(float rollDeg, float pitchDeg, float yawDeg,
                                        float fx, float fy, float cx, float cy) {
        return warpMatrix(
                Math.toRadians(rollDeg),
                Math.toRadians(pitchDeg),
                Math.toRadians(yawDeg),
                fx, fy, cx, cy);
    }

    /**
     * Same as flowpilot {@code CommonModelF3.transform_scale_buffer}: scale homography
     * for half-res chroma planes (s=0.5 → M_uv).
     */
    public static float[] transformScaleBuffer(float[] m, float s) {
        float invS = 1.0f / s;
        float[] transformOut = {
                invS, 0, 0.5f,
                0, invS, 0.5f,
                0, 0, 1
        };
        float[] transformIn = {
                s, 0, -0.5f * s,
                0, s, -0.5f * s,
                0, 0, 1
        };
        return mul3(transformIn, mul3(m, transformOut));
    }

    /**
     * Flowpilot TransformCL path on CPU: warp Y/U/V with M / M_uv into model I420
     * (Y 512×256, U/V 256×128). Matches GPU semantics (edge clamp, no RGB).
     */
    public static void warpYuvToModelI420(YuvFrame src, float[] mModelToCam, byte[] dstI420) {
        if (dstI420.length < MODEL_W * MODEL_H * 3 / 2) {
            throw new IllegalArgumentException("dstI420 too small");
        }
        float[] mUv = transformScaleBuffer(mModelToCam, 0.5f);
        int ySize = MODEL_W * MODEL_H;
        int uvSize = (MODEL_W / 2) * (MODEL_H / 2);
        warpPlane(src.y, src.width, src.height, mModelToCam,
                dstI420, 0, MODEL_W, MODEL_H, MODEL_W);
        warpPlane(src.u, src.uvWidth(), src.uvHeight(), mUv,
                dstI420, ySize, MODEL_W / 2, MODEL_H / 2, MODEL_W / 2);
        warpPlane(src.v, src.uvWidth(), src.uvHeight(), mUv,
                dstI420, ySize + uvSize, MODEL_W / 2, MODEL_H / 2, MODEL_W / 2);
    }

    /**
     * Warp YUV → supercombo 6-channel half-res float (same layout as parseImageYuvI420).
     * Prefers native C++ (−O3); falls back to Java.
     */
    public static void warpYuvToFrame6(YuvFrame src, float[] mModelToCam, float[] out6) {
        final int plane = (MODEL_W / 2) * (MODEL_H / 2);
        if (out6.length < 6 * plane) {
            throw new IllegalArgumentException("out6 too small");
        }
        if (ensureNativeWarp()) {
            try {
                if (nativeWarpYuvToFrame6(src.y, src.u, src.v, src.width, src.height, mModelToCam, out6)) {
                    return;
                }
            } catch (UnsatisfiedLinkError e) {
                nativeWarpOk = false;
            }
        }
        warpYuvToFrame6Java(src, mModelToCam, out6);
    }

    /** Java fallback (slow on 1280×720). */
    public static void warpYuvToFrame6Java(YuvFrame src, float[] mModelToCam, float[] out6) {
        final int ww = MODEL_W / 2;
        final int hh = MODEL_H / 2;
        final int plane = ww * hh;
        float[] m = mModelToCam;
        float[] mUv = transformScaleBuffer(m, 0.5f);
        final float m00 = m[0], m01 = m[1], m02 = m[2];
        final float m10 = m[3], m11 = m[4], m12 = m[5];
        final float m20 = m[6], m21 = m[7], m22 = m[8];
        final float u00 = mUv[0], u01 = mUv[1], u02 = mUv[2];
        final float u10 = mUv[3], u11 = mUv[4], u12 = mUv[5];
        final float u20 = mUv[6], u21 = mUv[7], u22 = mUv[8];
        final byte[] y = src.y;
        final byte[] u = src.u;
        final byte[] v = src.v;
        final int sw = src.width;
        final int sh = src.height;
        final int uvW = src.uvWidth();
        final int uvH = src.uvHeight();

        for (int j = 0; j < hh; j++) {
            for (int i = 0; i < ww; i++) {
                int idx = j * ww + i;
                int mx0 = 2 * i;
                int my0 = 2 * j;
                out6[0 * plane + idx] = sampleProj(y, sw, sh, m00, m01, m02, m10, m11, m12, m20, m21, m22, mx0, my0);
                out6[1 * plane + idx] = sampleProj(y, sw, sh, m00, m01, m02, m10, m11, m12, m20, m21, m22, mx0, my0 + 1);
                out6[2 * plane + idx] = sampleProj(y, sw, sh, m00, m01, m02, m10, m11, m12, m20, m21, m22, mx0 + 1, my0);
                out6[3 * plane + idx] = sampleProj(y, sw, sh, m00, m01, m02, m10, m11, m12, m20, m21, m22, mx0 + 1, my0 + 1);
                out6[4 * plane + idx] = sampleProj(u, uvW, uvH, u00, u01, u02, u10, u11, u12, u20, u21, u22, i, j);
                out6[5 * plane + idx] = sampleProj(v, uvW, uvH, u00, u01, u02, u10, u11, u12, u20, u21, u22, i, j);
            }
        }
    }

    private static float sampleProj(byte[] px, int w, int h,
                                    float m00, float m01, float m02,
                                    float m10, float m11, float m12,
                                    float m20, float m21, float m22,
                                    int x, int y) {
        float X = m00 * x + m01 * y + m02;
        float Y = m10 * x + m11 * y + m12;
        float W = m20 * x + m21 * y + m22;
        if (Math.abs(W) < 1e-8f) {
            return 0f;
        }
        return sampleBilinearU8f(px, w, h, X / W, Y / W);
    }

    private static void warpPlane(byte[] src, int srcW, int srcH, float[] m,
                                  byte[] dst, int dstOff, int dstW, int dstH, int dstStride) {
        final float m00 = m[0], m01 = m[1], m02 = m[2];
        final float m10 = m[3], m11 = m[4], m12 = m[5];
        final float m20 = m[6], m21 = m[7], m22 = m[8];
        for (int y = 0; y < dstH; y++) {
            int row = dstOff + y * dstStride;
            for (int x = 0; x < dstW; x++) {
                float X = m00 * x + m01 * y + m02;
                float Y = m10 * x + m11 * y + m12;
                float W = m20 * x + m21 * y + m22;
                if (Math.abs(W) < 1e-8f) {
                    dst[row + x] = 0;
                    continue;
                }
                dst[row + x] = (byte) (sampleBilinearU8f(src, srcW, srcH, X / W, Y / W) + 0.5f);
            }
        }
    }

    /** Bilinear sample; edge-clamp like TransformCL warpPerspective. */
    private static float sampleBilinearU8f(byte[] px, int w, int h, float sx, float sy) {
        if (sx < 0f) {
            sx = 0f;
        } else if (sx > w - 1) {
            sx = w - 1;
        }
        if (sy < 0f) {
            sy = 0f;
        } else if (sy > h - 1) {
            sy = h - 1;
        }
        int x0 = (int) Math.floor(sx);
        int y0 = (int) Math.floor(sy);
        int x1 = Math.min(x0 + 1, w - 1);
        int y1 = Math.min(y0 + 1, h - 1);
        float fx = sx - x0;
        float fy = sy - y0;
        int v00 = px[y0 * w + x0] & 0xff;
        int v10 = px[y0 * w + x1] & 0xff;
        int v01 = px[y1 * w + x0] & 0xff;
        int v11 = px[y1 * w + x1] & 0xff;
        float top = v00 + (v10 - v00) * fx;
        float bot = v01 + (v11 - v01) * fx;
        return top + (bot - top) * fy;
    }

    private static byte sampleBilinearU8(byte[] px, int w, int h, float sx, float sy) {
        return (byte) (sampleBilinearU8f(px, w, h, sx, sy) + 0.5f);
    }

    /**
     * Warp camera ARGB bitmap into model-sized ARGB using M (model→camera).
     * Legacy / debug path — production uses {@link #warpYuvToModelI420}.
     */
    public static Bitmap warpToModel(Bitmap src, float[] mModelToCam) {
        final int sw = src.getWidth();
        final int sh = src.getHeight();
        int[] srcPx = new int[sw * sh];
        src.getPixels(srcPx, 0, sw, 0, 0, sw, sh);

        int[] dstPx = new int[MODEL_W * MODEL_H];
        final float m00 = mModelToCam[0], m01 = mModelToCam[1], m02 = mModelToCam[2];
        final float m10 = mModelToCam[3], m11 = mModelToCam[4], m12 = mModelToCam[5];
        final float m20 = mModelToCam[6], m21 = mModelToCam[7], m22 = mModelToCam[8];

        for (int y = 0; y < MODEL_H; y++) {
            for (int x = 0; x < MODEL_W; x++) {
                float X = m00 * x + m01 * y + m02;
                float Y = m10 * x + m11 * y + m12;
                float W = m20 * x + m21 * y + m22;
                if (Math.abs(W) < 1e-8f) {
                    dstPx[y * MODEL_W + x] = 0xff000000;
                    continue;
                }
                float sx = X / W;
                float sy = Y / W;
                dstPx[y * MODEL_W + x] = sampleBilinear(srcPx, sw, sh, sx, sy);
            }
        }
        Bitmap out = Bitmap.createBitmap(MODEL_W, MODEL_H, Bitmap.Config.ARGB_8888);
        out.setPixels(dstPx, 0, MODEL_W, 0, 0, MODEL_W, MODEL_H);
        return out;
    }

    private static int sampleBilinear(int[] px, int w, int h, float sx, float sy) {
        if (sx < 0 || sy < 0 || sx >= w - 1 || sy >= h - 1) {
            if (sx < -0.5f || sy < -0.5f || sx >= w - 0.5f || sy >= h - 0.5f) {
                return 0xff000000;
            }
            int ix = clamp((int) Math.floor(sx + 0.5f), 0, w - 1);
            int iy = clamp((int) Math.floor(sy + 0.5f), 0, h - 1);
            return px[iy * w + ix];
        }
        int x0 = (int) Math.floor(sx);
        int y0 = (int) Math.floor(sy);
        float fx = sx - x0;
        float fy = sy - y0;
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int c00 = px[y0 * w + x0];
        int c10 = px[y0 * w + x1];
        int c01 = px[y1 * w + x0];
        int c11 = px[y1 * w + x1];
        return lerpArgb(lerpArgb(c00, c10, fx), lerpArgb(c01, c11, fx), fy);
    }

    private static int lerpArgb(int a, int b, float t) {
        int ar = (a >> 16) & 0xff, ag = (a >> 8) & 0xff, ab = a & 0xff;
        int br = (b >> 16) & 0xff, bg = (b >> 8) & 0xff, bb = b & 0xff;
        int r = (int) (ar + (br - ar) * t + 0.5f);
        int g = (int) (ag + (bg - ag) * t + 0.5f);
        int bl = (int) (ab + (bb - ab) * t + 0.5f);
        return 0xff000000 | (r << 16) | (g << 8) | bl;
    }

    /** Matches flowpilot Preprocess.eulerAnglesToRotationMatrix(..., isDegrees=false). */
    static float[] rotFromEuler(double roll, double pitch, double yaw) {
        float cp = (float) Math.cos(pitch);
        float sp = (float) Math.sin(pitch);
        float sr = (float) Math.sin(roll);
        float cr = (float) Math.cos(roll);
        float sy = (float) Math.sin(yaw);
        float cy = (float) Math.cos(yaw);

        // Same 3×3 as Preprocess before Nd4j transpose (row-major).
        float[] rot = {
                cp * cy, cp * sy, -sp,
                (sr * sp * cy) - (cr * sy), (sr * sp * sy) + (cr * cy), sr * cp,
                (cr * sp * cy) + (sr * sy), (cr * sp * sy) - (sr * cy), cr * cp
        };
        return transpose3(rot);
    }

    static float[] mul3(float[] a, float[] b) {
        float[] r = new float[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i * 3 + j] =
                        a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        return r;
    }

    static float[] transpose3(float[] m) {
        return new float[]{
                m[0], m[3], m[6],
                m[1], m[4], m[7],
                m[2], m[5], m[8]
        };
    }

    static float[] inv3(float[] m) {
        float a = m[0], b = m[1], c = m[2];
        float d = m[3], e = m[4], f = m[5];
        float g = m[6], h = m[7], i = m[8];
        float A = e * i - f * h;
        float B = f * g - d * i;
        float C = d * h - e * g;
        float det = a * A + b * B + c * C;
        if (Math.abs(det) < 1e-12f) {
            throw new IllegalArgumentException("singular 3x3");
        }
        float invDet = 1.0f / det;
        return new float[]{
                A * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet,
                B * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet,
                C * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet
        };
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
