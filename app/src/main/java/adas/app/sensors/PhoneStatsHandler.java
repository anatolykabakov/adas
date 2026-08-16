package adas.app.sensors;

import adas.app.Logger;
import adas.app.TimeUtil;
import adas.proto.Messages;

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.PowerManager;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import adas.proto.Messages.ZMQMessage;
import adas.proto.PhoneStatsOuter.PhoneStats;
import adas.proto.PhoneStatsOuter.ThermalZone;

/**
 * 1 Hz phone CPU / thermal sampler → bag topic {@code phone/stats}.
 * Bag-only (not published to native). Used to correlate vision stalls with SoC heat.
 */
public final class PhoneStatsHandler {
    private static final String TAG = "PhoneStatsHandler";
    private static final long PERIOD_MS = 1000L;

    private final Context appContext;
    private final PowerManager powerManager;

    private HandlerThread thread;
    private Handler handler;
    private boolean running;

    private long prevTotalJiffies = -1;
    private long prevIdleJiffies = -1;
    private long prevAppJiffies = -1;

    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            if (!running) {
                return;
            }
            try {
                sampleAndLog();
            } catch (Exception e) {
                Log.w(TAG, "sample failed", e);
            }
            if (running && handler != null) {
                handler.postDelayed(this, PERIOD_MS);
            }
        }
    };

    public PhoneStatsHandler(Context context) {
        this.appContext = context.getApplicationContext();
        this.powerManager = (PowerManager) appContext.getSystemService(Context.POWER_SERVICE);
    }

    public synchronized void start() {
        if (running) {
            return;
        }
        thread = new HandlerThread("PhoneStats");
        thread.start();
        handler = new Handler(thread.getLooper());
        running = true;
        // Prime /proc counters; first publish after one period.
        readCpuCounters();
        handler.postDelayed(tick, PERIOD_MS);
        Log.i(TAG, "started (1 Hz → phone/stats)");
    }

    public synchronized void stop() {
        if (!running) {
            return;
        }
        running = false;
        if (handler != null) {
            handler.removeCallbacks(tick);
            handler = null;
        }
        if (thread != null) {
            thread.quitSafely();
            thread = null;
        }
        prevTotalJiffies = -1;
        prevIdleJiffies = -1;
        prevAppJiffies = -1;
        Log.i(TAG, "stopped");
    }

    private void sampleAndLog() {
        long ts = TimeUtil.nowMs();
        float[] cpu = readCpuPct();
        int thermal = readThermalStatus();
        float[] batt = readBattery();
        List<ThermalZone> zones = readThermalZones();
        float cpuTemp = pickTemp(zones, new String[]{"cpu", "soc", "tsens", "msm-therm"});
        float skinTemp = pickTemp(zones, new String[]{"skin", "xo-therm", "quiet-therm"});
        float gpuTemp = pickTemp(zones, new String[]{"gpu", "gpuss"});
        int freqKhz = readCpu0FreqKhz();

        PhoneStats.Builder b = PhoneStats.newBuilder()
                .setTimestamp(ts)
                .setCpuPct(cpu[0])
                .setCpuAppPct(cpu[1])
                .setThermalStatus(thermal)
                .setBatteryTempC(batt[0])
                .setBatteryPct(batt[1])
                .setCpuTempC(cpuTemp)
                .setSkinTempC(skinTemp)
                .setGpuTempC(gpuTemp)
                .setCpu0FreqKhz(freqKhz);
        for (ThermalZone z : zones) {
            b.addZones(z);
        }

        ZMQMessage msg = Messages.ZMQMessage.newBuilder()
                .setTimestamp(ts)
                .setTopic("phone/stats")
                .setPhoneStats(b.build())
                .build();
        Logger.getInstance().logZMQMessage(msg);
    }

    /** @return {system_pct, app_pct}; 0 if first sample / read fail. */
    private float[] readCpuPct() {
        long[] cur = readCpuCounters();
        if (cur == null) {
            return new float[]{0f, 0f};
        }
        long total = cur[0];
        long idle = cur[1];
        long app = cur[2];
        float sysPct = 0f;
        float appPct = 0f;
        if (prevTotalJiffies >= 0) {
            long dTotal = total - prevTotalJiffies;
            long dIdle = idle - prevIdleJiffies;
            long dApp = app - prevAppJiffies;
            if (dTotal > 0) {
                sysPct = 100f * (1f - (float) dIdle / (float) dTotal);
                appPct = 100f * (float) dApp / (float) dTotal;
                if (sysPct < 0f) {
                    sysPct = 0f;
                }
                if (sysPct > 100f) {
                    sysPct = 100f;
                }
                if (appPct < 0f) {
                    appPct = 0f;
                }
                if (appPct > 100f) {
                    appPct = 100f;
                }
            }
        }
        prevTotalJiffies = total;
        prevIdleJiffies = idle;
        prevAppJiffies = app;
        return new float[]{sysPct, appPct};
    }

    /** @return {total, idle, appJiffies} or null. */
    private long[] readCpuCounters() {
        long total = -1;
        long idle = -1;
        try (BufferedReader br = new BufferedReader(new FileReader("/proc/stat"))) {
            String line = br.readLine();
            if (line == null || !line.startsWith("cpu ")) {
                return null;
            }
            String[] p = line.trim().split("\\s+");
            // cpu user nice system idle iowait irq softirq steal guest guest_nice
            if (p.length < 5) {
                return null;
            }
            long user = Long.parseLong(p[1]);
            long nice = Long.parseLong(p[2]);
            long system = Long.parseLong(p[3]);
            long idleJ = Long.parseLong(p[4]);
            long iowait = p.length > 5 ? Long.parseLong(p[5]) : 0;
            long irq = p.length > 6 ? Long.parseLong(p[6]) : 0;
            long softirq = p.length > 7 ? Long.parseLong(p[7]) : 0;
            long steal = p.length > 8 ? Long.parseLong(p[8]) : 0;
            idle = idleJ + iowait;
            total = user + nice + system + idle + irq + softirq + steal;
        } catch (Exception e) {
            return null;
        }
        long app = 0;
        try (BufferedReader br = new BufferedReader(new FileReader("/proc/self/stat"))) {
            String line = br.readLine();
            if (line != null) {
                // comm may contain spaces inside (); split after last ')'
                int close = line.lastIndexOf(')');
                String rest = close >= 0 ? line.substring(close + 1).trim() : line;
                String[] p = rest.split("\\s+");
                // after comm: state(0) … utime(11) stime(12)  — 0-based in rest
                if (p.length > 12) {
                    app = Long.parseLong(p[11]) + Long.parseLong(p[12]);
                }
            }
        } catch (Exception ignored) {
        }
        return new long[]{total, idle, app};
    }

    private int readThermalStatus() {
        if (powerManager == null || Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            return -1;
        }
        try {
            return powerManager.getCurrentThermalStatus();
        } catch (Exception e) {
            return -1;
        }
    }

    /** @return {temp_c, pct} */
    private float[] readBattery() {
        float tempC = 0f;
        float pct = 0f;
        try {
            Intent bat = appContext.registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
            if (bat != null) {
                int tTenths = bat.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0);
                tempC = tTenths / 10f;
                int level = bat.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
                int scale = bat.getIntExtra(BatteryManager.EXTRA_SCALE, -1);
                if (level >= 0 && scale > 0) {
                    pct = 100f * level / scale;
                }
            }
        } catch (Exception ignored) {
        }
        return new float[]{tempC, pct};
    }

    private static List<ThermalZone> readThermalZones() {
        List<ThermalZone> out = new ArrayList<>();
        File root = new File("/sys/class/thermal");
        File[] dirs = root.listFiles();
        if (dirs == null) {
            return out;
        }
        for (File dir : dirs) {
            if (!dir.isDirectory() || !dir.getName().startsWith("thermal_zone")) {
                continue;
            }
            String type = readFirstLine(new File(dir, "type"));
            String tempStr = readFirstLine(new File(dir, "temp"));
            if (type == null || tempStr == null) {
                continue;
            }
            try {
                float milli = Float.parseFloat(tempStr.trim());
                // Some kernels report °C already (< 200); most use millidegC.
                float c = milli > 200f ? milli / 1000f : milli;
                if (c < -20f || c > 120f) {
                    continue;
                }
                out.add(ThermalZone.newBuilder()
                        .setName(type.trim())
                        .setTempC(c)
                        .build());
            } catch (NumberFormatException ignored) {
            }
        }
        return out;
    }

    private static float pickTemp(List<ThermalZone> zones, String[] needles) {
        float best = 0f;
        boolean found = false;
        for (ThermalZone z : zones) {
            String name = z.getName().toLowerCase(Locale.US);
            for (String n : needles) {
                if (name.contains(n)) {
                    if (!found || z.getTempC() > best) {
                        best = z.getTempC();
                        found = true;
                    }
                    break;
                }
            }
        }
        return best;
    }

    private static int readCpu0FreqKhz() {
        String s = readFirstLine(new File("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"));
        if (s == null) {
            return 0;
        }
        try {
            return Integer.parseInt(s.trim());
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    private static String readFirstLine(File f) {
        if (!f.isFile()) {
            return null;
        }
        try (BufferedReader br = new BufferedReader(new FileReader(f))) {
            return br.readLine();
        } catch (Exception e) {
            return null;
        }
    }
}
