package adas.app.vision.overlay;

/** Device frame → screen, owned by the view. */
public interface GroundProjector {
    /**
     * \param x Forward [m], device frame.
     * \param y Right [m].
     * \param z Down [m] — the road plane sits at the camera height.
     * \param xMin Points closer than this are behind any useful drawing.
     * \param outPt Screen x,y on success.
     * \return False when the point is behind the camera or far off screen.
     */
    boolean project(float x, float y, float z, float xMin, float[] outPt);
}
