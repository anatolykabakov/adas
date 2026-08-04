package ai.flow.adas;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.Arrays;

import ai.flow.adas.Messages.ZMQMessage;

public class IMUHandler implements SensorEventListener {

    private static final String TAG = "IMUHandler";
    private static final int SENSOR_DELAY = SensorManager.SENSOR_DELAY_GAME;

    private final SensorManager sensorManager;
    private boolean isRunning = false;

    private Sensor accelerometer;
    private Sensor gyroscope;
    private Sensor magnetometer;

    private final float[] accelerometerData = new float[3];
    private final float[] gyroscopeData = new float[3];
    private final float[] magnetometerData = new float[3];
    private int sampleCount = 0;

    public IMUHandler(Context context) {
        this.sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        }
    }

    public void start() {
        if (isRunning) {
            Log.w(TAG, "IMU handler already running");
            return;
        }

        if (sensorManager == null) {
            Log.e(TAG, "Sensor manager not available");
            return;
        }

        try {
            if (accelerometer != null) {
                sensorManager.registerListener(this, accelerometer, SENSOR_DELAY);
            } else {
                Log.w(TAG, "Accelerometer not available");
            }
            if (gyroscope != null) {
                sensorManager.registerListener(this, gyroscope, SENSOR_DELAY);
            } else {
                Log.w(TAG, "Gyroscope not available");
            }
            if (magnetometer != null) {
                sensorManager.registerListener(this, magnetometer, SENSOR_DELAY);
            } else {
                Log.w(TAG, "Magnetometer not available");
            }
            isRunning = true;
            Log.i(TAG, "IMU handler started");
        } catch (SecurityException e) {
            Log.e(TAG, "HIGH_SAMPLING_RATE_SENSORS denied; trying SENSOR_DELAY_NORMAL", e);
            try {
                if (accelerometer != null) {
                    sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
                }
                if (gyroscope != null) {
                    sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_NORMAL);
                }
                if (magnetometer != null) {
                    sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
                }
                isRunning = true;
                Log.i(TAG, "IMU handler started with fallback sampling rate");
            } catch (Exception fallbackException) {
                Log.e(TAG, "Failed to start IMU handler", fallbackException);
            }
        } catch (Exception e) {
            Log.e(TAG, "Error starting IMU handler", e);
        }
    }

    public void stop() {
        if (!isRunning) {
            return;
        }
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
        isRunning = false;
        Log.i(TAG, "IMU handler stopped");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event == null || event.values == null) {
            return;
        }

        sampleCount++;
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, accelerometerData, 0, 3);
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(event.values, 0, gyroscopeData, 0, 3);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values, 0, magnetometerData, 0, 3);
                break;
            default:
                return;
        }

        if (sampleCount % 10 != 0) {
            return;
        }

        try {
            ZMQMessage imuMessage = ProtoUtils.createIMUDataMessage(
                    Arrays.asList(accelerometerData[0], accelerometerData[1], accelerometerData[2]),
                    Arrays.asList(gyroscopeData[0], gyroscopeData[1], gyroscopeData[2]),
                    Arrays.asList(magnetometerData[0], magnetometerData[1], magnetometerData[2]),
                    TimeUtil.nowMs());
            imuMessage = imuMessage.toBuilder().setTopic("sensors/imu").build();
            ZMQBridgeService.publishToNative(imuMessage);
            Logger.getInstance().logZMQMessage(imuMessage);
        } catch (Exception e) {
            Log.e(TAG, "Error publishing IMU message", e);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        Log.d(TAG, "Sensor " + sensor.getName() + " accuracy changed: " + accuracy);
    }
}
