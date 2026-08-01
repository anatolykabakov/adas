package ai.flow.adas;

import android.util.Log;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

public class Logger {
    private static final String TAG = "Logger";

    private static volatile Logger instance;
    private static final Object lock = new Object();

    private final AtomicBoolean running = new AtomicBoolean(false);
    private File logDirectory;
    private volatile BagLogger bagLogger;

    private Logger() {
    }

    public static Logger getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new Logger();
                }
            }
        }
        return instance;
    }

    public void start(String basePath) {
        if (running.get()) {
            Log.i(TAG, "Logger already running");
            return;
        }

        Log.i(TAG, "Starting Logger...");

        String ts = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", Locale.US).format(new Date());
        File dir = new File(basePath, ts);
        if (!dir.exists()) {
            boolean created = dir.mkdirs();
            if (!created) {
                Log.e(TAG, "Failed to create log directory: " + dir.getAbsolutePath());
                return;
            }
            Log.i(TAG, "Created new log directory: " + dir.getAbsolutePath());
        } else {
            Log.i(TAG, "Using existing log directory: " + dir.getAbsolutePath());
        }

        logDirectory = dir;
        BagLogger bl = BagLogger.getInstance();
        bl.startInDirectory(dir);
        bagLogger = bl;
        running.set(true);

        Log.i(TAG, "Logger started successfully");
    }

    public void stop() {
        if (!running.get()) {
            return;
        }

        running.set(false);

        BagLogger bl = bagLogger;
        bagLogger = null;
        if (bl != null) {
            bl.stop();
        }

        Log.i(TAG, "Logger stopped. Log directory preserved: "
                + (logDirectory != null ? logDirectory.getAbsolutePath() : "null"));
    }

    public boolean isRunning() {
        return running.get();
    }

    public void logZMQMessage(Messages.ZMQMessage message) {
        if (message == null || !running.get()) {
            return;
        }
        BagLogger bl = bagLogger;
        if (bl == null) {
            return;
        }
        bl.addMessage(message.getTopic(), message);
    }
}
