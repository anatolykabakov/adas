package adas.app;

import adas.app.bridge.ZMQBridgeService;
import android.app.PendingIntent;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import org.json.JSONObject;
import android.os.IBinder;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.HashMap;

class AdasAppInstance implements Runnable {
    private final int fd;
    private final String dbcPath;
    private final String configPath;

    public AdasAppInstance(int fd, String dbcPath, String configPath) {
        this.fd = fd;
        this.dbcPath = dbcPath;
        this.configPath = configPath;
    }

    @Override
    public void run() {
        if (!AdasAppHandler.nativeLoaded) {
            Log.w("AdasAppHandler", "Skipping nativeStart: libadas_app_android.so not loaded");
            return;
        }
        try {
            AdasAppHandler.nativeStart(this.fd, this.dbcPath, this.configPath);
            AdasAppHandler.flushPendingLaneKeepParams();
        } catch (Throwable t) {
            Log.e("AdasAppHandler", "nativeStart failed (fd=" + fd + ")", t);
        }
    }
}

public class AdasAppHandler extends Service {
    String TAG = "AdasAppHandler";

    public static final boolean nativeLoaded;

    private static final String ACTION_USB_PERMISSION = "adas.app.USB_PERMISSION";
    /** Fallback DBC when `vehicle.name` is unset in the config or unknown. */
    private static final String DBC_ASSET_FALLBACK = "vw_mqb_2010.dbc";

    private static final int PANDA_VID = 0xbbaa;
    private static final int PANDA_PID = 0xddcc;

    private String dbcPath;
    private String configPath;

    private UsbDeviceConnection pandaConnection;
    private boolean nativeStarted;
    /** The descriptor the native side was started with. -1 means "no panda". */
    private int startedFd = -1;

    /** Watchdog for a panda that stopped answering. */
    private static final long PANDA_DEAD_MS = 3000;
    /** Time after a (re)start before the watchdog may judge: the native side needs a few seconds. */
    private static final long PANDA_GRACE_MS = 12000;
    /** Between attempts. Re-opening a USB device is not free and the cause may be outside our reach. */
    private static final long PANDA_RETRY_MS = 5000;
    private static final long PANDA_TICK_MS = 2000;
    /** How many times to try seating a new descriptor before restarting the native side instead. */
    private static final int PANDA_RESEAT_MAX = 2;

    private final android.os.Handler pandaWatchdog =
            new android.os.Handler(android.os.Looper.getMainLooper());
    private long lastPandaActionMs;
    private int pandaReconnects;
    /** Seat attempts since the last time health actually arrived. */
    private int reseatStreak;

    private BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            Log.d(TAG, "USB intent: " + action);
            if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
                synchronized (this) {
                    UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    maybeRequestUSBPermission(usbDevice, context);
                }
            } else if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    maybeRequestUSBPermission(usbDevice, context);
                }
            }
        }
    };

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "AdasAppHandler created (nativeLoaded=" + nativeLoaded + ")");

        // The DBC is chosen by the car rather than hardcoded: a different make means a different bus layout.
        final String carName = AdasConfig.carName(this);
        String dbcAsset = AdasConfig.dbcAssetFor(carName);
        if (dbcAsset.isEmpty()) {
            Log.w(TAG, "vehicle.name='" + carName + "' not recognised, taking " + DBC_ASSET_FALLBACK);
            dbcAsset = DBC_ASSET_FALLBACK;
        } else {
            Log.i(TAG, "car " + carName + ", DBC " + dbcAsset);
        }
        dbcPath = ensureAssetCopied(this, dbcAsset, /*force=*/false);
        configPath = ensureAssetCopied(this, AdasConfig.ASSET, /*force=*/false);
        Log.i(TAG, "DBC path: " + dbcPath);
        Log.i(TAG, "Config path: " + configPath);

        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(ACTION_USB_PERMISSION);

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            registerReceiver(usbReceiver, filter, Context.RECEIVER_EXPORTED);
        } else {
            registerReceiver(usbReceiver, filter);
        }

        pandaWatchdog.postDelayed(pandaWatchdogTick, PANDA_TICK_MS);

        new android.os.Handler(android.os.Looper.getMainLooper()).postDelayed(() -> {
            UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
            if (manager != null) {
                HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
                Log.i(TAG, "Number of USB devices found: " + deviceList.size());
                for (UsbDevice usbDevice : deviceList.values()) {
                    maybeRequestUSBPermission(usbDevice, this);
                }
            }
            // No panda — start without it. The planner, localisation, calibration, vision and
            // logging do not depend on the hardware, and waiting for a panda would mean having no
            // test bench at all without a car.
            startNative(-1);
        }, 1500);

        Intent zmqIntent = new Intent(this, ZMQBridgeService.class);
        startService(zmqIntent);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "AdasAppHandler starting...");
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.i(TAG, "AdasAppHandler destroying...");

        // Before anything else: a watchdog that outlives the service would try to restart the native
        // side of a dead process.
        pandaWatchdog.removeCallbacks(pandaWatchdogTick);

        if (usbReceiver != null) {
            unregisterReceiver(usbReceiver);
        }

        if (nativeLoaded && nativeStarted) {
            try {
                nativeStop();
            } catch (Throwable t) {
                Log.w(TAG, "nativeStop failed", t);
            }
        }

        if (pandaConnection != null) {
            try {
                pandaConnection.close();
            } catch (Exception e) {
                Log.w(TAG, "Error closing panda USB connection", e);
            }
            pandaConnection = null;
        }
        nativeStarted = false;
        startedFd = -1;

        Intent zmqIntent = new Intent(this, ZMQBridgeService.class);
        stopService(zmqIntent);
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private static boolean isPanda(UsbDevice device) {
        return device != null && device.getVendorId() == PANDA_VID && device.getProductId() == PANDA_PID;
    }

    private final Runnable pandaWatchdogTick = new Runnable() {
        @Override
        public void run() {
            try {
                checkPandaAlive();
            } catch (Throwable t) {
                Log.w(TAG, "panda watchdog", t);   // a watchdog that dies is worse than none
            }
            pandaWatchdog.postDelayed(this, PANDA_TICK_MS);
        }
    };

    /** If the panda has gone quiet, take the descriptor again. */
    private synchronized void checkPandaAlive() {
        if (!nativeStarted || startedFd == -1) {
            return;
        }
        final long now = android.os.SystemClock.elapsedRealtime();
        if (now - lastPandaActionMs < PANDA_GRACE_MS) {
            return;
        }
        final long lastHealth = adas.app.bridge.ZMQBridgeService.lastPandaHealthElapsedMs();
        // Never seen a health message at all: the grace period above already gave it time, so this is
        // the same fault as having lost it.
        final long silentMs = lastHealth == 0 ? now - lastPandaActionMs : now - lastHealth;
        if (silentMs < PANDA_DEAD_MS) {
            // Health is flowing, so whatever we did last time worked: the next fault starts from zero.
            reseatStreak = 0;
            return;
        }
        if (now - lastPandaActionMs < PANDA_RETRY_MS) {
            return;
        }

        final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            return;
        }
        UsbDevice panda = null;
        for (UsbDevice d : usbManager.getDeviceList().values()) {
            if (isPanda(d)) {
                panda = d;
                break;
            }
        }
        if (panda == null) {
            // Physically gone. Nothing to re-open; the attach broadcast will bring us back.
            Log.w(TAG, "panda silent " + silentMs + " ms and not in the device list — waiting for attach");
            lastPandaActionMs = now;
            return;
        }
        if (!usbManager.hasPermission(panda)) {
            Log.w(TAG, "panda silent " + silentMs + " ms but permission is gone — asking again");
            lastPandaActionMs = now;
            maybeRequestUSBPermission(panda, this);
            return;
        }

        Log.w(TAG, "panda silent " + silentMs + " ms — reopening the descriptor (attempt "
                + (pandaReconnects + 1) + ")");
        // The old connection has to go first, or openDevice hands back the same dead one.
        if (pandaConnection != null) {
            try {
                pandaConnection.close();
            } catch (Throwable t) {
                Log.w(TAG, "closing the stale panda connection", t);
            }
            pandaConnection = null;
        }
        final UsbDeviceConnection conn = usbManager.openDevice(panda);
        lastPandaActionMs = now;
        if (conn == null) {
            Log.e(TAG, "openDevice returned null — will try again in " + PANDA_RETRY_MS + " ms");
            return;
        }
        pandaConnection = conn;
        pandaReconnects++;
        final int fd = conn.getFileDescriptor();

        // Hand the descriptor to the panda driver alone. Everything else — planner, controller, pose
        // estimator, paramsd — keeps running with the state it has built up during the drive; the driver
        // is usually about to pull away, which is the worst moment to be back on default parameters.
        if (reseatStreak < PANDA_RESEAT_MAX && reseatPandaNative(fd)) {
            reseatStreak++;
            startedFd = fd;
            Log.i(TAG, "panda reopened, fd=" + fd + " — seated into the running native side"
                    + " (reconnect #" + pandaReconnects + ", seat " + reseatStreak + "/" + PANDA_RESEAT_MAX + ")");
            return;
        }

        Log.w(TAG, "panda reopened, fd=" + fd + " — "
                + (reseatStreak >= PANDA_RESEAT_MAX ? "seating it twice did not bring the board back"
                                                    : "the native side would not take it")
                + ", restarting native (reconnect #" + pandaReconnects + ")");
        reseatStreak = 0;
        // Force the restart: after closing the old connection the kernel usually hands back the *same*
        // descriptor number, and `startNative` treats an unchanged fd as "nothing to do".
        restartNativeWith(fd);
    }

    private void maybeRequestUSBPermission(UsbDevice usbDevice, Context context) {
        if (!isPanda(usbDevice)) {
            if (usbDevice != null) {
                Log.d(TAG, "Skipping non-panda USB device VID=0x"
                        + Integer.toHexString(usbDevice.getVendorId())
                        + " PID=0x" + Integer.toHexString(usbDevice.getProductId()));
            }
            return;
        }

        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        if (usbManager.hasPermission(usbDevice)) {
            Log.i(TAG, "USB permission already granted for panda: " + usbDevice.getDeviceName());
            openPandaAndStartNative(usbManager, usbDevice);
        } else {
            Log.i(TAG, "Requesting USB permission for panda: " + usbDevice.getDeviceName());
            PendingIntent permissionIntent = PendingIntent.getBroadcast(context, 0, new Intent(ACTION_USB_PERMISSION),
                    PendingIntent.FLAG_IMMUTABLE);
            usbManager.requestPermission(usbDevice, permissionIntent);
        }
    }

    private synchronized void openPandaAndStartNative(UsbManager usbManager, UsbDevice usbDevice) {
        if (nativeStarted && startedFd != -1) {
            Log.i(TAG, "Native panda already started — skip duplicate open");
            return;
        }
        UsbDeviceConnection conn = usbManager.openDevice(usbDevice);
        if (conn == null) {
            Log.e(TAG, "openDevice returned null for panda");
            return;
        }

        pandaConnection = conn;
        int fd = conn.getFileDescriptor();
        Log.i(TAG, "USB Device FD: " + fd + " (connection retained)");
        startNative(fd);
    }

    /** Stop and start the native side on this descriptor, even if the number has not changed. */
    private synchronized void restartNativeWith(int fd) {
        startedFd = -1;
        startNative(fd);
    }

    private synchronized void startNative(int fd) {
        if (nativeStarted) {
            if (fd == -1 || startedFd == fd) {
                return;
            }
            Log.i(TAG, "Panda arrived after start — restarting native with fd=" + fd);
            if (nativeLoaded) {
                try {
                    nativeStop();
                } catch (Throwable t) {
                    Log.w(TAG, "nativeStop failed", t);
                }
            }
        }
        nativeStarted = true;
        startedFd = fd;
        lastPandaActionMs = android.os.SystemClock.elapsedRealtime();
        Log.i(TAG, fd == -1 ? "Starting native without a panda (bench mode)" : "Starting native with panda fd=" + fd);
        new Thread(new AdasAppInstance(fd, dbcPath, configPath), "AdasNative").start();
    }

    static String ensureAssetCopied(Context context, String assetName, boolean force) {
        File out = new File(context.getFilesDir(), assetName);
        if (!force && out.exists() && out.length() > 0) {
            return out.getAbsolutePath();
        }
        try (InputStream in = context.getAssets().open(assetName);
             FileOutputStream fos = new FileOutputStream(out)) {
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) {
                fos.write(buf, 0, n);
            }
            Log.i("AdasAppHandler", "Copied asset " + assetName + " -> " + out.getAbsolutePath());
            return out.getAbsolutePath();
        } catch (Exception e) {
            Log.e("AdasAppHandler", "Failed to copy asset " + assetName, e);
            return out.exists() ? out.getAbsolutePath() : "";
        }
    }

    /** @param mapPath absolute path to the OSM road map, or "" to leave the configured one alone. */
    public static native void nativeStart(int fd, String dbcPath, String configPath);

    public static native void nativeStop();

    public static native int nativeUpdateParams(String jsonParams);

    /**
     * Seat a re-opened panda descriptor in the running native side.
     *
     * @return false when the native side is down or has no panda service — the caller must restart then.
     */
    public static native boolean nativeReseatPanda(int fd);

    /** As above, but a missing symbol or a native throw costs a restart, not the watchdog. */
    private static boolean reseatPandaNative(int fd) {
        try {
            return nativeReseatPanda(fd);
        } catch (Throwable t) {
            Log.w("AdasAppHandler", "nativeReseatPanda unavailable — falling back to a restart", t);
            return false;
        }
    }

    private static volatile RuntimeParams pendingLaneKeepParams;

    public static void applyLaneKeepParams(RuntimeParams p) {
        if (p == null) {
            return;
        }
        pendingLaneKeepParams = p;
        flushPendingLaneKeepParams();
    }

    static void flushPendingLaneKeepParams() {
        RuntimeParams p = pendingLaneKeepParams;
        if (!nativeLoaded || p == null) {
            return;
        }
        try {
            JSONObject params = new JSONObject();
            params.put("pp_k_dd", p.ppKdd);
            params.put("pp_ld_min", p.ppLdMin);
            params.put("pp_ld_max", p.ppLdMax);
            params.put("pp_shift", p.ppShift);
            params.put("steer_ratio", p.steerRatio);
            params.put("cam_y_left_m", p.camY);
            params.put("path_lane_blend_scale", p.laneBlendScale);
            String ctrl = p.laneKeepController;
            params.put("lane_keep_controller", ctrl == null || ctrl.isEmpty() ? "pp" : ctrl);
            final int applied = nativeUpdateParams(params.toString());
            if (applied < 0) {
                Log.i("AdasAppHandler", "params deferred until nativeStart");
            } else if (applied < params.length()) {
                Log.w("AdasAppHandler", "applied " + applied + " of " + params.length() + " params");
            }
        } catch (Throwable t) {
            Log.w("AdasAppHandler", "applyLaneKeepParams failed", t);
        }
    }

    static {
        boolean ok = false;
        try {
            System.loadLibrary("adas_app_android");
            ok = true;
            Log.i("AdasAppHandler", "Loaded libadas_app_android.so");
        } catch (UnsatisfiedLinkError e) {
            Log.e("AdasAppHandler", "libadas_app_android.so missing; native panda path disabled", e);
        }
        nativeLoaded = ok;
    }
}
