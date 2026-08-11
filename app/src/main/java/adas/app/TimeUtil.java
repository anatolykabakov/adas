package adas.app;

import android.os.SystemClock;

public final class TimeUtil {
    private TimeUtil() {}

    public static long nowMs() {
        return SystemClock.elapsedRealtime();
    }
}
