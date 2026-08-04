package ai.flow.adas.vision;

import android.media.Image;

import java.nio.ByteBuffer;

/**
 * Contiguous copy of a Camera2 {@link android.graphics.ImageFormat#YUV_420_888} frame.
 * Planes are tightly packed (pixel stride 1) so TransformCL-style warps can sample them.
 */
public final class YuvFrame {
    public final int width;
    public final int height;
    /** Y plane: width * height */
    public final byte[] y;
    /** U plane: (width/2) * (height/2) */
    public final byte[] u;
    /** V plane: (width/2) * (height/2) */
    public final byte[] v;

    public YuvFrame(int width, int height, byte[] y, byte[] u, byte[] v) {
        this.width = width;
        this.height = height;
        this.y = y;
        this.u = u;
        this.v = v;
    }

    /** Deep copy for a second consumer (e.g. low-freq traffic YOLO). */
    public YuvFrame duplicate() {
        return new YuvFrame(width, height, y.clone(), u.clone(), v.clone());
    }

    public int uvWidth() {
        return width / 2;
    }

    public int uvHeight() {
        return height / 2;
    }

    /** Deep-copy planes from {@code image}. Call before {@code image.close()}. */
    public static YuvFrame copyFrom(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        int uvW = width / 2;
        int uvH = height / 2;
        Image.Plane[] planes = image.getPlanes();
        ByteBuffer yBuf = planes[0].getBuffer();
        ByteBuffer uBuf = planes[1].getBuffer();
        ByteBuffer vBuf = planes[2].getBuffer();
        int yRowStride = planes[0].getRowStride();
        int yPxStride = planes[0].getPixelStride();
        int uRowStride = planes[1].getRowStride();
        int uPxStride = planes[1].getPixelStride();
        int vRowStride = planes[2].getRowStride();
        int vPxStride = planes[2].getPixelStride();

        byte[] y = new byte[width * height];
        byte[] u = new byte[uvW * uvH];
        byte[] v = new byte[uvW * uvH];

        copyPlaneTight(yBuf, width, height, yRowStride, yPxStride, y);

        // NV12: U/V interleaved in one buffer (pixel stride 2). Bulk-copy then deinterleave.
        if (uPxStride == 2 && vPxStride == 2) {
            byte[] uvRow = new byte[uRowStride];
            ByteBuffer uDup = uBuf.duplicate();
            ByteBuffer vDup = vBuf.duplicate();
            boolean sameUv = (uBuf == vBuf)
                    || (uBuf.capacity() == vBuf.capacity() && uRowStride == vRowStride);
            for (int row = 0; row < uvH; row++) {
                int dst = row * uvW;
                int uPos = row * uRowStride;
                uDup.clear();
                int uLim = Math.min(uDup.capacity(), uPos + uRowStride);
                if (uPos < uLim) {
                    uDup.position(uPos);
                    uDup.limit(uLim);
                    int n = Math.min(uRowStride, uDup.remaining());
                    uDup.get(uvRow, 0, n);
                }
                // V plane may be same NV12 buffer with +1 offset, or separate.
                if (sameUv && vPxStride == 2) {
                    for (int col = 0; col < uvW; col++) {
                        int i = col * 2;
                        u[dst + col] = uvRow[i];
                        v[dst + col] = uvRow[i + 1];
                    }
                } else {
                    for (int col = 0; col < uvW; col++) {
                        u[dst + col] = uvRow[col * uPxStride];
                    }
                    for (int col = 0; col < uvW; col++) {
                        v[dst + col] = vDup.get(row * vRowStride + col * vPxStride);
                    }
                }
            }
        } else {
            copyPlaneTight(uBuf, uvW, uvH, uRowStride, uPxStride, u);
            copyPlaneTight(vBuf, uvW, uvH, vRowStride, vPxStride, v);
        }
        return new YuvFrame(width, height, y, u, v);
    }

    private static void copyPlaneTight(ByteBuffer src, int width, int height,
                                       int rowStride, int pxStride, byte[] dst) {
        ByteBuffer buf = src.duplicate();
        if (pxStride == 1) {
            for (int row = 0; row < height; row++) {
                int srcPos = row * rowStride;
                int dstPos = row * width;
                buf.clear();
                if (srcPos + width > buf.capacity()) {
                    for (int col = 0; col < width; col++) {
                        dst[dstPos + col] = buf.get(srcPos + col);
                    }
                    continue;
                }
                buf.position(srcPos);
                buf.limit(srcPos + width);
                buf.get(dst, dstPos, width);
            }
            return;
        }
        for (int row = 0; row < height; row++) {
            int srcRow = row * rowStride;
            int dstPos = row * width;
            for (int col = 0; col < width; col++) {
                dst[dstPos + col] = buf.get(srcRow + col * pxStride);
            }
        }
    }
}
