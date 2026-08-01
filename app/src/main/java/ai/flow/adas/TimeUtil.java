package ai.flow.adas;

import android.os.SystemClock;

public final class TimeUtil {
    private TimeUtil() {}

    public static long nowMs() {
        return SystemClock.elapsedRealtime();
    }
}
