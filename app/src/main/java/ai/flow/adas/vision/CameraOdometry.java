package ai.flow.adas.vision;

public final class CameraOdometry {

    public static final int POSE_SIZE = 12;
    public static final int TEMPORAL_SIZE = 512;
    public static final int OUTPUT_SIZE = 6409;
    public static final int POSE_IDX = OUTPUT_SIZE - TEMPORAL_SIZE - POSE_SIZE;

    public final float[] trans = new float[3];
    public final float[] rot = new float[3];
    public final float[] transStd = new float[3];
    public final float[] rotStd = new float[3];
    public boolean valid;

    public static int poseIdx(int outputLen) {
        if (outputLen < POSE_SIZE + TEMPORAL_SIZE) {
            return Math.max(0, outputLen - POSE_SIZE);
        }
        return outputLen - TEMPORAL_SIZE - POSE_SIZE;
    }

    public static CameraOdometry parse(float[] out) {
        CameraOdometry o = new CameraOdometry();
        if (out == null) {
            return o;
        }
        int poseIdx = poseIdx(out.length);
        if (out.length < poseIdx + POSE_SIZE) {
            return o;
        }
        o.trans[0] = out[poseIdx];
        o.trans[1] = out[poseIdx + 1];
        o.trans[2] = out[poseIdx + 2];
        o.rot[0] = out[poseIdx + 3];
        o.rot[1] = out[poseIdx + 4];
        o.rot[2] = out[poseIdx + 5];
        for (int i = 0; i < 3; i++) {
            o.transStd[i] = (float) Math.exp(out[poseIdx + 6 + i]);
            o.rotStd[i] = (float) Math.exp(out[poseIdx + 9 + i]);
        }
        o.valid = Float.isFinite(o.trans[0]) && Float.isFinite(o.trans[1]) && Float.isFinite(o.trans[2]);
        return o;
    }
}
