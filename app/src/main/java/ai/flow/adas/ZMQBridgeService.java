package ai.flow.adas;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import org.zeromq.ZContext;
import org.zeromq.ZMQ;
import org.zeromq.ZMQ.Poller;
import org.zeromq.ZMQ.Socket;

import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import ai.flow.adas.Messages.ZMQMessage;

public class ZMQBridgeService extends Service {
    private static final String TAG = "ZMQBridgeService";

    public static final String ENDPOINT_IN = "tcp://127.0.0.1:5555";
    public static final String ENDPOINT_OUT = "tcp://127.0.0.1:5556";

    public interface OutboundListener {
        void onOutbound(String topic, ZMQMessage message);
    }

    private static final AtomicReference<ZMQBridgeService> INSTANCE = new AtomicReference<>();
    private static final List<OutboundListener> LISTENERS = new CopyOnWriteArrayList<>();
    private static final AtomicLong LAST_PANDA_HEALTH_ELAPSED_MS = new AtomicLong(0);

    private final Object pubLock = new Object();
    private final Object lifecycleLock = new Object();

    private ZContext context;
    private Socket pubIn;
    private Socket subOut;
    private Poller poller;
    private ExecutorService zmqExecutor;
    private volatile boolean isRunning = false;

    public static ZMQBridgeService getInstance() {
        return INSTANCE.get();
    }

    public static void addOutboundListener(OutboundListener listener) {
        if (listener != null) {
            LISTENERS.add(listener);
        }
    }

    public static void removeOutboundListener(OutboundListener listener) {
        LISTENERS.remove(listener);
    }

    /** ElapsedRealtime ms of last panda/health; 0 if never. */
    public static long lastPandaHealthElapsedMs() {
        return LAST_PANDA_HEALTH_ELAPSED_MS.get();
    }

    @Override
    public void onCreate() {
        super.onCreate();
        INSTANCE.set(this);
        Log.i(TAG, "ZMQBridgeService created");
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "ZMQBridgeService starting...");
        startZMQBridge();
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.i(TAG, "ZMQBridgeService destroying...");
        stopZMQBridge();
        INSTANCE.compareAndSet(this, null);
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private void startZMQBridge() {
        synchronized (lifecycleLock) {
            if (isRunning) {
                Log.i(TAG, "ZMQ Bridge already running — skip restart");
                return;
            }
            try {
                setupZMQ();
                zmqExecutor = Executors.newSingleThreadExecutor();
                isRunning = true;
                zmqExecutor.submit(this::messageLoop);
                Log.i(TAG, "ZMQ Bridge started (PUB→" + ENDPOINT_IN + ", SUB←" + ENDPOINT_OUT + ")");
            } catch (Exception e) {
                Log.e(TAG, "Error starting ZMQ Bridge", e);
                cleanupSocketsUnlocked();
                isRunning = false;
            }
        }
    }

    private void stopZMQBridge() {
        synchronized (lifecycleLock) {
            Log.i(TAG, "Stopping ZMQ Bridge...");
            isRunning = false;

            if (zmqExecutor != null) {
                zmqExecutor.shutdown();
                try {
                    if (!zmqExecutor.awaitTermination(5, TimeUnit.SECONDS)) {
                        zmqExecutor.shutdownNow();
                    }
                } catch (InterruptedException e) {
                    zmqExecutor.shutdownNow();
                    Thread.currentThread().interrupt();
                }
                zmqExecutor = null;
            }

            synchronized (pubLock) {
                cleanupSocketsUnlocked();
            }

            Log.i(TAG, "ZMQ Bridge stopped");
        }
    }

    private void cleanupSocketsUnlocked() {
        if (context != null) {
            try {
                context.close();
            } catch (Exception e) {
                Log.w(TAG, "Error closing ZContext", e);
            }
            context = null;
        }
        pubIn = null;
        subOut = null;
        poller = null;
    }

    private void setupZMQ() {
        context = new ZContext();

        pubIn = context.createSocket(ZMQ.PUB);
        pubIn.connect(ENDPOINT_IN);
        pubIn.setSendTimeOut(10);
        Log.i(TAG, "ZMQ PUB connected to " + ENDPOINT_IN);

        subOut = context.createSocket(ZMQ.SUB);
        subOut.connect(ENDPOINT_OUT);
        subOut.subscribe("".getBytes(StandardCharsets.UTF_8));
        subOut.setReceiveTimeOut(10);
        Log.i(TAG, "ZMQ SUB connected to " + ENDPOINT_OUT);

        poller = context.createPoller(1);
        poller.register(subOut, Poller.POLLIN);
    }

    private void messageLoop() {
        Log.d(TAG, "ZMQ message loop started");
        while (isRunning) {
            try {
                Poller p = poller;
                if (p == null) {
                    Thread.sleep(10);
                    continue;
                }
                int events = p.poll(10);
                if (events > 0 && p.pollin(0)) {
                    recvOutbound();
                }
            } catch (Exception e) {
                if (isRunning) {
                    Log.e(TAG, "Error in ZMQ message loop", e);
                }
            }
        }
        Log.d(TAG, "ZMQ message loop stopped");
    }

    private void recvOutbound() {
        try {
            Socket sub = subOut;
            if (sub == null) {
                return;
            }

            byte[] first = sub.recv(ZMQ.DONTWAIT);
            if (first == null || first.length == 0) {
                return;
            }

            String topic;
            byte[] payload;
            if (sub.hasReceiveMore()) {
                topic = new String(first, StandardCharsets.UTF_8);
                payload = sub.recv(ZMQ.DONTWAIT);
                if (payload == null) {
                    Log.w(TAG, "Outbound multipart missing payload for " + topic);
                    return;
                }
            } else {
                payload = first;
                topic = null;
            }

            ZMQMessage zmqMsg = ZMQMessage.parseFrom(payload);
            if (topic == null || topic.isEmpty()) {
                topic = zmqMsg.getTopic();
            }
            Log.d(TAG, "ZMQ outbound '" + topic + "' " + payload.length + " bytes");
            Logger.getInstance().logZMQMessage(zmqMsg);
            if ("panda/health".equals(topic) || zmqMsg.hasPandaHealth()) {
                LAST_PANDA_HEALTH_ELAPSED_MS.set(android.os.SystemClock.elapsedRealtime());
            }
            for (OutboundListener l : LISTENERS) {
                try {
                    l.onOutbound(topic, zmqMsg);
                } catch (Exception e) {
                    Log.w(TAG, "OutboundListener error", e);
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "Error receiving outbound ZMQ", e);
        }
    }

    public void publishInternalMessage(String topic, ZMQMessage message) {
        if (!isRunning) {
            Log.w(TAG, "ZMQ Bridge not running");
            return;
        }
        synchronized (pubLock) {
            if (!isRunning || pubIn == null) {
                Log.w(TAG, "ZMQ Bridge not running");
                return;
            }
            try {
                ZMQMessage.Builder b = message.toBuilder();
                if (b.getTopic().isEmpty() && topic != null) {
                    b.setTopic(topic);
                }
                byte[] body = b.build().toByteArray();
                String t = (topic != null && !topic.isEmpty()) ? topic : b.getTopic();
                boolean ok = pubIn.sendMore(t.getBytes(StandardCharsets.UTF_8))
                        && pubIn.send(body, ZMQ.DONTWAIT);
                if (!ok) {
                    Log.d(TAG, "ZMQ inbound send dropped for '" + t + "'");
                }
            } catch (Exception e) {
                Log.e(TAG, "Error publishing to native for topic: " + topic, e);
            }
        }
    }

    /** Convenience for sensor handlers: topic taken from {@link ZMQMessage#getTopic()}. */
    public static void publishToNative(ZMQMessage message) {
        ZMQBridgeService svc = INSTANCE.get();
        if (svc == null) {
            Log.w(TAG, "ZMQBridgeService not started; drop inbound");
            return;
        }
        String topic = message.getTopic();
        svc.publishInternalMessage(topic, message);
    }

    public boolean isRunning() {
        return isRunning;
    }
}
