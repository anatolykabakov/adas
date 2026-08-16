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
    private final String mapPath;

    public AdasAppInstance(int fd, String dbcPath, String configPath, String mapPath) {
        this.fd = fd;
        this.dbcPath = dbcPath;
        this.configPath = configPath;
        this.mapPath = mapPath;
    }

    @Override
    public void run() {
        if (!AdasAppHandler.nativeLoaded) {
            Log.w("AdasAppHandler", "Skipping nativeStart: libadas_app_android.so not loaded");
            return;
        }
        try {
            AdasAppHandler.nativeStart(this.fd, this.dbcPath, this.configPath, this.mapPath);
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
    private String mapPath;

    private UsbDeviceConnection pandaConnection;
    private boolean nativeStarted;
    /** The descriptor the native side was started with. -1 means "no panda". */
    private int startedFd = -1;

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

        // The road map is 4.9 MB, so it is unpacked only when the service that reads it is on. Copying it
        // unconditionally would cost every install that storage for a node that is off by default.
        if (AdasConfig.mapDataEnabled(this)) {
            mapPath = ensureAssetCopied(this, AdasConfig.mapAsset(this), /*force=*/false);
            Log.i(TAG, "Map path: " + mapPath);
        } else {
            mapPath = "";
            Log.i(TAG, "map_data node off — road map not unpacked");
        }

        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(ACTION_USB_PERMISSION);

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            registerReceiver(usbReceiver, filter, Context.RECEIVER_EXPORTED);
        } else {
            registerReceiver(usbReceiver, filter);
        }

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

    /** Start the native side on this descriptor.
     *
     *  The panda may arrive after the bench does: a live process running without one is then stopped
     *  and started again with the descriptor. Car-side services are created once at startup and
     *  cannot be handed hardware on the fly — a restart is honester than half a living app. */
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
        Log.i(TAG, fd == -1 ? "Starting native without a panda (bench mode)" : "Starting native with panda fd=" + fd);
        new Thread(new AdasAppInstance(fd, dbcPath, configPath, mapPath), "AdasNative").start();
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
    public static native void nativeStart(int fd, String dbcPath, String configPath, String mapPath);

    public static native void nativeStop();

    public static native int nativeUpdateParams(String jsonParams);

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
