package adas.app;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;

import androidx.core.app.ActivityCompat;

import adas.proto.Messages.ZMQMessage;

public class GPSHandler implements LocationListener {

    private static final String TAG = "GPSHandler";
    private static final long MIN_TIME_MS = 1000;
    private static final float MIN_DISTANCE_M = 1.0f;

    private final Context context;
    private final LocationManager locationManager;
    private boolean isRunning = false;

    public GPSHandler(Context context) {
        this.context = context;
        this.locationManager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
    }

    public void start() {
        if (isRunning) {
            Log.w(TAG, "GPS handler already running");
            return;
        }

        if (locationManager == null) {
            Log.e(TAG, "Location manager not available");
            return;
        }

        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
                        != PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_COARSE_LOCATION)
                        != PackageManager.PERMISSION_GRANTED) {
            Log.e(TAG, "Location permissions not granted");
            return;
        }

        try {
            locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER, MIN_TIME_MS, MIN_DISTANCE_M, this);
            locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER, MIN_TIME_MS, MIN_DISTANCE_M, this);
            isRunning = true;
            Log.i(TAG, "GPS handler started");
        } catch (SecurityException e) {
            Log.e(TAG, "Security exception when requesting location updates", e);
        }
    }

    public void stop() {
        if (!isRunning) {
            return;
        }
        if (locationManager != null) {
            locationManager.removeUpdates(this);
        }
        isRunning = false;
        Log.i(TAG, "GPS handler stopped");
    }

    @Override
    public void onLocationChanged(Location location) {
        if (location == null) {
            return;
        }

        long currentTime = TimeUtil.nowMs();
        try {
            final float accuracy = location.hasAccuracy() ? location.getAccuracy() : 0.0f;
            int satellites = 0;
            if (location.getExtras() != null) {
                satellites = location.getExtras().getInt("satellites", 0);
            }
            ZMQMessage locationMessage = ProtoUtils.createGPSLocationMessage(
                    location.getLatitude(),
                    location.getLongitude(),
                    location.getAltitude(),
                    location.getSpeed(),
                    location.getBearing(),
                    currentTime,
                    accuracy,
                    satellites,
                    LocationManager.GPS_PROVIDER.equals(location.getProvider()));
            locationMessage = locationMessage.toBuilder().setTopic("sensors/gps/location").build();

            ZMQMessage dataMessage = ProtoUtils.createGPSDataMessage(
                    location.getLatitude(),
                    location.getLongitude(),
                    location.getAltitude(),
                    location.getSpeed(),
                    location.getBearing(),
                    currentTime);
            dataMessage = dataMessage.toBuilder().setTopic("sensors/gps/data").build();

            ZMQBridgeService.publishToNative(locationMessage);
            Logger.getInstance().logZMQMessage(locationMessage);
            Logger.getInstance().logZMQMessage(dataMessage);
        } catch (Exception e) {
            Log.e(TAG, "Error publishing GPS messages", e);
        }
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        Log.d(TAG, "GPS provider " + provider + " status changed: " + status);
    }

    @Override
    public void onProviderEnabled(String provider) {
        Log.d(TAG, "GPS provider " + provider + " enabled");
    }

    @Override
    public void onProviderDisabled(String provider) {
        Log.d(TAG, "GPS provider " + provider + " disabled");
    }
}
