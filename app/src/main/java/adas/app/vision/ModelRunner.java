package adas.app.vision;

/**
 * What the pipeline knows about the vision model. Two implementations exist: supercombo 0.8.12 via
 * ONNX Runtime, and the flowpilot 0.9.x model on the GPU via thneed. Selected by
 * {@code vision.model_runner}.
 *
 * <p>Inference timing is part of the contract, not a side log: implementations are only comparable
 * through the same measurement. It goes into {@link LaneLines#inferDurationMs} and into the bag.
 */
public interface ModelRunner {

    /** @return parsed output, or null when skipped: both implementations need a previous frame. */
    SupercomboOnnxRunner.Result run(YuvFrame frame, int frameId, long captureTsMs) throws Exception;

    /**
     * Legacy Bitmap path. Unsupported by default: GPU implementations take a flat buffer, and a second
     * preparation path nobody exercises is worse than none.
     */
    default SupercomboOnnxRunner.Result run(android.graphics.Bitmap bitmap, int frameId, long captureTsMs)
            throws Exception {
        throw new UnsupportedOperationException(name() + " does not take bitmaps");
    }

    void setCalib(float rollDeg, float pitchDeg, float yawDeg,
                  float fx, float fy, float cx, float cy, int width, int height);

    /**
     * Vehicle speed [m/s]. No-op by default: supercombo 0.8.12 has no such input. The 0.9.x model does,
     * and a constant zero there would be out of distribution on every frame. Source is
     * {@code vehicle/state} from CAN, not the model's own pose estimate.
     */
    default void setEgoSpeed(float speedMps) {
    }

    void close();

    /** Name for the log and the bag, so a drive can be attributed to a model. */
    String name();
}
