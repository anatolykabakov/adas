package ai.flow.adas.vision;

/** Device/openpilot frame: X forward, Y right-positive, Z up (flowpilot Parser). */
public class LaneLines {
    public static final int N = 33;


    public static final float[] X_IDXS = new float[]{
            0.f, 0.1875f, 0.75f, 1.6875f, 3.f, 4.6875f,
            6.75f, 9.1875f, 12.f, 15.1875f, 18.75f, 22.6875f,
            27.f, 31.6875f, 36.75f, 42.1875f, 48.f, 54.1875f,
            60.75f, 67.6875f, 75.f, 82.6875f, 90.75f, 99.1875f,
            108.f, 117.1875f, 126.75f, 136.6875f, 147.f, 157.6875f,
            168.75f, 180.1875f, 192.f
    };


    public final float[][] lanesY = new float[4][N];

    public final float[][] lanesZ = new float[4][N];

    /** Per-point lateral sigma of each lane line (m), from the model's own std block.
     *  Zero means "not filled" — consumers must treat that as "no information", not as certainty. */
    public final float[][] lanesYStd = new float[4][N];

    public final float[][] edgesY = new float[2][N];
    public final float[][] edgesZ = new float[2][N];
    /** Road edge sigma (m), same exp() convention as the lane sigmas. */
    public final float[][] edgesYStd = new float[2][N];
    public final float[] laneProbs = new float[4];


    public final float[] planX = new float[N];
    public final float[] planY = new float[N];
    public final float[] planZ = new float[N];
    /** PLAN orientation.z (rad), device frame — flowpilot modelV2.orientation.z. */
    public final float[] planYaw = new float[N];
    /** PLAN orientationRate.z (rad/s) — flowpilot modelV2.orientationRate.z. */
    public final float[] planYawRate = new float[N];
    public int planHypIndex = -1;
    public boolean hasPlan;

    public long timestampMs;       // capture (primary)
    public long captureTimestampMs;
    public long inferTimestampMs;
    /** Wall time of OrtSession.run only (ms). */
    public float inferDurationMs;
    /** Warp + pack to 6ch (ms), before session.run. */
    public float prepDurationMs;
    public int frameId;

    /** When the frame reached {@code VisionPipeline.submitYuv} — camera to app delivery. */
    public long submitTimestampMs;
    /** When the inference thread took it out of the 1-slot latest buffer — queue wait. */
    public long pickupTimestampMs;
    /**
     * Captures overwritten in that buffer since the previous processed frame, i.e. thrown away
     * because inference was still running. Zero means the pipeline kept up with the camera.
     */
    public int framesDropped;

    /** Full ONNX flat output for bag offline debug; null if not set. */
    public float[] modelOut;

    // CIPV / long summary (filled from ModelLongParse; also vision/model_long).
    public float leadD;
    public float leadY;
    public float leadV;
    public float leadProb;
    public float planV0;
    public boolean leadValid;

    public LaneLines copy() {
        LaneLines o = new LaneLines();
        for (int i = 0; i < 4; i++) {
            System.arraycopy(lanesY[i], 0, o.lanesY[i], 0, N);
            System.arraycopy(lanesZ[i], 0, o.lanesZ[i], 0, N);
            System.arraycopy(lanesYStd[i], 0, o.lanesYStd[i], 0, N);
            o.laneProbs[i] = laneProbs[i];
        }
        for (int i = 0; i < 2; i++) {
            System.arraycopy(edgesY[i], 0, o.edgesY[i], 0, N);
            System.arraycopy(edgesZ[i], 0, o.edgesZ[i], 0, N);
            System.arraycopy(edgesYStd[i], 0, o.edgesYStd[i], 0, N);
        }
        System.arraycopy(planX, 0, o.planX, 0, N);
        System.arraycopy(planY, 0, o.planY, 0, N);
        System.arraycopy(planZ, 0, o.planZ, 0, N);
        System.arraycopy(planYaw, 0, o.planYaw, 0, N);
        System.arraycopy(planYawRate, 0, o.planYawRate, 0, N);
        o.planHypIndex = planHypIndex;
        o.hasPlan = hasPlan;
        o.timestampMs = timestampMs;
        o.captureTimestampMs = captureTimestampMs;
        o.inferTimestampMs = inferTimestampMs;
        o.inferDurationMs = inferDurationMs;
        o.prepDurationMs = prepDurationMs;
        o.frameId = frameId;
        o.submitTimestampMs = submitTimestampMs;
        o.pickupTimestampMs = pickupTimestampMs;
        o.framesDropped = framesDropped;
        if (modelOut != null) {
            o.modelOut = modelOut.clone();
        }
        o.leadD = leadD;
        o.leadY = leadY;
        o.leadV = leadV;
        o.leadProb = leadProb;
        o.planV0 = planV0;
        o.leadValid = leadValid;
        return o;
    }
}
