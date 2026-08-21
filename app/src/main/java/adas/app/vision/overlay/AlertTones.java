package adas.app.vision.overlay;

import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Handler;
import android.os.Looper;

/** Audible alerts. */
public final class AlertTones {
    private static final long REPEAT_MS = 700;

    private ToneGenerator toneGen;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private volatile boolean repeating;
    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            if (!repeating) {
                return;
            }
            beep(ToneGenerator.TONE_CDMA_ABBR_ALERT, 400);
            handler.postDelayed(this, REPEAT_MS);
        }
    };

    /** Critical repeats until {@link #stop}; a warning speaks once. */
    public void play(boolean critical) {
        handler.removeCallbacks(tick);
        if (critical) {
            repeating = true;
            handler.post(tick);
        } else {
            repeating = false;
            beep(ToneGenerator.TONE_PROP_BEEP2, 250);
        }
    }

    /** Silence everything — for leaving the road, stopping the pipeline, or losing the view. */
    public void stop() {
        repeating = false;
        handler.removeCallbacks(tick);
        if (toneGen != null) {
            toneGen.stopTone();
        }
    }

    /** The view can go away mid-alert. A tone that outlives it is worse than no tone at all. */
    public void release() {
        stop();
        if (toneGen != null) {
            toneGen.release();
            toneGen = null;
        }
    }

    private void beep(int tone, int durationMs) {
        try {
            if (toneGen == null) {
                toneGen = new ToneGenerator(AudioManager.STREAM_ALARM, 90);
            }
            toneGen.startTone(tone, durationMs);
        } catch (RuntimeException e) {
            // No audio focus, or the stream is taken. A missing beep must not take the display with it.
            toneGen = null;
        }
    }
}
