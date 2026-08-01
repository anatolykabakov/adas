package ai.flow.adas;

import java.util.List;
import bag.BagOuterClass;
import ai.flow.adas.Messages;
import ai.flow.adas.Imu;
import ai.flow.adas.Gps;
import ai.flow.adas.Camera;
import ai.flow.adas.CameraIntrinsicsOuterClass;
import ai.flow.adas.Lanes;

public class ProtoUtils {

    public static BagOuterClass.Bag createBagMessage(List<Messages.ZMQMessage> zmqMessages) {
        BagOuterClass.Bag.Builder bagBuilder = BagOuterClass.Bag.newBuilder();

        if (!zmqMessages.isEmpty()) {

            com.google.protobuf.Timestamp timestamp = com.google.protobuf.util.Timestamps.fromMillis(zmqMessages.get(0).getTimestamp());
            bagBuilder.setTimestamp(timestamp);

            bagBuilder.addAllMessages(zmqMessages);
        }

        return bagBuilder.build();
    }

    public static String fixTopicName(String topicName) {
        if (topicName == null) return "";
        return topicName.replace("/", "__");
    }

    public static String createDataFileName(long timestamp) {
        java.text.SimpleDateFormat sdf = new java.text.SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", java.util.Locale.US);
        return sdf.format(new java.util.Date(timestamp)) + ".bin";
    }

    public static boolean isValidForBagLogging(Messages.ZMQMessage message) {
        if (message == null) return false;

        String topic = message.getTopic();
        if (topic == null || topic.isEmpty()) return false;

        return message.hasCameraImage() ||
               message.hasGpsLocation() ||
               message.hasGpsData() ||
               message.hasImuData() ||
               message.hasCanData() ||
               message.hasPandaHealth() ||
               message.hasCameraIntrinsics() ||
               message.hasLaneLines() ||
               message.hasCarState() ||
               message.hasSteerCommand() ||
               message.hasLaneKeep() ||
               message.hasLocalizationPose() ||
               message.hasCameraCalib() ||
               message.hasLaneUv() ||
               message.hasCameraOdometry() ||
               message.hasMiddlewareStats() ||
               message.hasLaneKeepDebug() ||
               message.hasModelLongPlan() ||
               message.hasLongPlan() ||
               message.hasSafetyWarn() ||
               message.hasCameraCalibDebug() ||
               message.hasTrafficDetections() ||
               message.hasTrafficVision() ||
               message.hasPhoneStats();
    }

    public static int getMessageSize(Messages.ZMQMessage message) {
        if (message == null) return 0;
        return message.getSerializedSize();
    }

    public static Messages.ZMQMessage createIMUDataMessage(java.util.List<Float> accel,
                                                          java.util.List<Float> gyro,
                                                          java.util.List<Float> mag,
                                                          long timestamp) {
        Messages.ZMQMessage zmqMessage = Messages.ZMQMessage.newBuilder()
            .setTimestamp(timestamp)
            .setTopic("sensors/imu")
            .build();

        Imu.IMUData.Builder imuBuilder = Imu.IMUData.newBuilder();
        imuBuilder.setTimestamp(timestamp);

        if (accel != null && accel.size() >= 3) {
            imuBuilder.setAccelX(accel.get(0));
            imuBuilder.setAccelY(accel.get(1));
            imuBuilder.setAccelZ(accel.get(2));
        }

        if (gyro != null && gyro.size() >= 3) {
            imuBuilder.setGyroX(gyro.get(0));
            imuBuilder.setGyroY(gyro.get(1));
            imuBuilder.setGyroZ(gyro.get(2));
        }

        if (mag != null && mag.size() >= 3) {
            imuBuilder.setMagX(mag.get(0));
            imuBuilder.setMagY(mag.get(1));
            imuBuilder.setMagZ(mag.get(2));
        }

        imuBuilder.setSampleCount(1);

        return zmqMessage.toBuilder()
            .setImuData(imuBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createGPSLocationMessage(double latitude, double longitude,
                                                              double altitude, float speed,
                                                              float bearing, long timestamp) {
        Messages.ZMQMessage zmqMessage = Messages.ZMQMessage.newBuilder()
            .setTimestamp(timestamp)
            .setTopic("sensors/gps/location")
            .build();

        Gps.GPSLocation.Builder gpsBuilder = Gps.GPSLocation.newBuilder();
        gpsBuilder.setTimestamp(timestamp);
        gpsBuilder.setLatitude(latitude);
        gpsBuilder.setLongitude(longitude);
        gpsBuilder.setAltitude(altitude);
        gpsBuilder.setSpeed(speed);
        gpsBuilder.setBearing(bearing);
        gpsBuilder.setFixType(Gps.GPSLocation.FixType.FIX_3D);

        return zmqMessage.toBuilder()
            .setGpsLocation(gpsBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createGPSDataMessage(double latitude, double longitude,
                                                          double altitude, float speed,
                                                          float bearing, long timestamp) {
        Messages.ZMQMessage zmqMessage = Messages.ZMQMessage.newBuilder()
            .setTimestamp(timestamp)
            .setTopic("sensors/gps/data")
            .build();

        Gps.GPSData.Builder gpsBuilder = Gps.GPSData.newBuilder();
        gpsBuilder.setTimestamp(timestamp);
        gpsBuilder.setLatitude(latitude);
        gpsBuilder.setLongitude(longitude);
        gpsBuilder.setAltitude(altitude);
        gpsBuilder.setSpeed(speed);
        gpsBuilder.setBearing(bearing);
        gpsBuilder.setGpsTimestamp(timestamp);
        gpsBuilder.setProvider("gps");

        return zmqMessage.toBuilder()
            .setGpsData(gpsBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createCameraImageMessage(java.util.List<Byte> imageData,
                                                              int width, int height,
                                                              String format, int frameId,
                                                              long timestamp,
                                                              float fx, float fy, float cx, float cy) {
        byte[] imageBytes = new byte[imageData.size()];
        for (int i = 0; i < imageData.size(); i++) {
            imageBytes[i] = imageData.get(i);
        }
        return createCameraImageMessage(imageBytes, width, height, format, frameId, timestamp,
                fx, fy, cx, cy);
    }

    public static Messages.ZMQMessage createCameraImageMessage(byte[] imageData,
                                                              int width, int height,
                                                              String format, int frameId,
                                                              long timestamp,
                                                              float fx, float fy, float cx, float cy) {
        Messages.ZMQMessage zmqMessage = Messages.ZMQMessage.newBuilder()
            .setTimestamp(timestamp)
            .setTopic("sensors/camera/image")
            .build();

        Camera.CameraImage.Builder cameraBuilder = Camera.CameraImage.newBuilder();
        cameraBuilder.setTimestamp(timestamp);
        cameraBuilder.setWidth(width);
        cameraBuilder.setHeight(height);
        cameraBuilder.setFormat(format);
        cameraBuilder.setFrameId(frameId);
        cameraBuilder.setFocalLengthX(fx);
        cameraBuilder.setFocalLengthY(fy);
        cameraBuilder.setPrincipalPointX(cx);
        cameraBuilder.setPrincipalPointY(cy);
        if (imageData != null) {
            cameraBuilder.setImageData(com.google.protobuf.ByteString.copyFrom(imageData));
        }
        cameraBuilder.setCameraType(Camera.CameraImage.CameraType.WIDE_ROAD);

        return zmqMessage.toBuilder()
            .setCameraImage(cameraBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createCameraIntrinsicsMessage(
            float physicalFocalLengthMm,
            float sensorWidthMm, float sensorHeightMm,
            int activeArrayWidth, int activeArrayHeight,
            float[] distortionCoefficients,
            float[] intrinsicCalibration,
            float focalLengthPx,
            int captureWidth, int captureHeight,
            String cameraId, String distortionModel,
            long timestamp) {

        Messages.ZMQMessage zmqMessage = Messages.ZMQMessage.newBuilder()
            .setTimestamp(timestamp)
            .setTopic("camera/intrinsics")
            .build();

        CameraIntrinsicsOuterClass.CameraIntrinsics.Builder intrinsicsBuilder = CameraIntrinsicsOuterClass.CameraIntrinsics.newBuilder();
        intrinsicsBuilder.setTimestamp(timestamp);
        intrinsicsBuilder.setPhysicalFocalLengthMm(physicalFocalLengthMm);
        intrinsicsBuilder.setSensorWidthMm(sensorWidthMm);
        intrinsicsBuilder.setSensorHeightMm(sensorHeightMm);
        intrinsicsBuilder.setActiveArrayWidth(activeArrayWidth);
        intrinsicsBuilder.setActiveArrayHeight(activeArrayHeight);

        if (distortionCoefficients != null) {
            for (float coeff : distortionCoefficients) {
                intrinsicsBuilder.addDistortionCoefficients(coeff);
            }
        }

        if (intrinsicCalibration != null) {
            for (float calib : intrinsicCalibration) {
                intrinsicsBuilder.addIntrinsicCalibration(calib);
            }
        }

        intrinsicsBuilder.setFocalLengthPx(focalLengthPx);
        intrinsicsBuilder.setCaptureWidth(captureWidth);
        intrinsicsBuilder.setCaptureHeight(captureHeight);
        intrinsicsBuilder.setCameraId(cameraId);
        intrinsicsBuilder.setDistortionModel(distortionModel);

        return zmqMessage.toBuilder()
            .setCameraIntrinsics(intrinsicsBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createLaneLinesMessage(ai.flow.adas.vision.LaneLines ll) {
        return createLaneLinesMessage(ll, /*includeModelOut=*/true);
    }

    public static Messages.ZMQMessage createLaneLinesMessage(
            ai.flow.adas.vision.LaneLines ll, boolean includeModelOut) {
        if (ll == null) {
            return null;
        }

        Lanes.LaneLines.Builder lanesBuilder = Lanes.LaneLines.newBuilder()
            .setTimestamp(ll.timestampMs)
            .setFrameId(ll.frameId)
            .setCaptureTsMs(ll.captureTimestampMs > 0 ? ll.captureTimestampMs : ll.timestampMs)
            .setInferTsMs(ll.inferTimestampMs)
            .setInferDurationMs(ll.inferDurationMs);

        for (float x : ai.flow.adas.vision.LaneLines.X_IDXS) {
            lanesBuilder.addX(x);
        }
        for (int i = 0; i < 4; i++) {
            Lanes.LanePolyline.Builder poly = Lanes.LanePolyline.newBuilder().setProb(ll.laneProbs[i]);
            for (float y : ll.lanesY[i]) {
                poly.addY(y);
            }
            for (float z : ll.lanesZ[i]) {
                poly.addZ(z);
            }
            for (float std : ll.lanesYStd[i]) {
                poly.addYStd(std);
            }
            lanesBuilder.addLanes(poly.build());
        }
        for (int i = 0; i < 2; i++) {
            Lanes.LanePolyline.Builder poly = Lanes.LanePolyline.newBuilder();
            for (float y : ll.edgesY[i]) {
                poly.addY(y);
            }
            for (float z : ll.edgesZ[i]) {
                poly.addZ(z);
            }
            lanesBuilder.addEdges(poly.build());
        }
        if (ll.hasPlan) {
            for (float x : ll.planX) {
                lanesBuilder.addPlanX(x);
            }
            for (float y : ll.planY) {
                lanesBuilder.addPlanY(y);
            }
            for (float z : ll.planZ) {
                lanesBuilder.addPlanZ(z);
            }
            for (float yaw : ll.planYaw) {
                lanesBuilder.addPlanYaw(yaw);
            }
            for (float yr : ll.planYawRate) {
                lanesBuilder.addPlanYawRate(yr);
            }
            lanesBuilder.setPlanHyp(ll.planHypIndex);
        }
        lanesBuilder.setLeadD(ll.leadD)
                .setLeadY(ll.leadY)
                .setLeadV(ll.leadV)
                .setLeadProb(ll.leadProb)
                .setPlanV0(ll.planV0)
                .setLeadValid(ll.leadValid);
        if (includeModelOut && ll.modelOut != null) {
            for (float v : ll.modelOut) {
                lanesBuilder.addModelOut(v);
            }
        }

        return Messages.ZMQMessage.newBuilder()
            .setTimestamp(ll.timestampMs)
            .setTopic("vision/lanes")
            .setLaneLines(lanesBuilder.build())
            .build();
    }

    public static Messages.ZMQMessage createCameraOdometryMessage(
            long timestampMs, int frameId, ai.flow.adas.vision.CameraOdometry pose) {
        if (pose == null || !pose.valid) {
            return null;
        }
        CameraCalibOuter.CameraOdometry.Builder o =
                CameraCalibOuter.CameraOdometry.newBuilder()
                        .setTimestamp(timestampMs)
                        .setFrameId(frameId);
        for (int i = 0; i < 3; i++) {
            o.addTrans(pose.trans[i]);
            o.addRot(pose.rot[i]);
            o.addTransStd(pose.transStd[i]);
            o.addRotStd(pose.rotStd[i]);
        }
        return Messages.ZMQMessage.newBuilder()
                .setTimestamp(timestampMs)
                .setTopic("model/camera_odometry")
                .setCameraOdometry(o.build())
                .build();
    }

    public static Messages.ZMQMessage createModelLongPlanMessage(
            long timestampMs, int frameId, ai.flow.adas.vision.ModelLongParse.Out longOut,
            ai.flow.adas.vision.CameraOdometry pose) {
        if (longOut == null || !longOut.ok) {
            return null;
        }
        ModelLongOuter.ModelLongPlan.Builder b = ModelLongOuter.ModelLongPlan.newBuilder()
                .setTimestamp(timestampMs)
                .setFrameId(frameId)
                .setPlanV0(longOut.planVx[0]);
        for (float v : longOut.planVx) {
            b.addPlanVX(v);
        }
        for (float v : longOut.planVy) {
            b.addPlanVY(v);
        }
        for (float v : longOut.planVz) {
            b.addPlanVZ(v);
        }
        b.setLead0(toLeadTrack(longOut.lead0));
        b.setLead1(toLeadTrack(longOut.lead1));
        b.setLead2(toLeadTrack(longOut.lead2));
        if (pose != null && pose.valid) {
            b.setPoseVx(pose.trans[0]);
            b.setPoseVy(pose.trans[1]);
            b.setPoseVz(pose.trans[2]);
            b.setPoseValid(true);
        }
        return Messages.ZMQMessage.newBuilder()
                .setTimestamp(timestampMs)
                .setTopic("vision/model_long")
                .setModelLongPlan(b.build())
                .build();
    }

    public static Messages.ZMQMessage createTrafficDetectionsMessage(
            ai.flow.adas.vision.TrafficYoloRunner.Result res, int frameId, long captureTsMs) {
        long inferTs = ai.flow.adas.TimeUtil.nowMs();
        TrafficVisionOuter.TrafficDetections.Builder b =
                TrafficVisionOuter.TrafficDetections.newBuilder()
                        .setTimestamp(inferTs)
                        .setFrameId(frameId)
                        .setModel(res.model == null ? "" : res.model)
                        .setInferMs(res.inferMs)
                        .setPrepMs(res.prepMs)
                        .setOrtMs(res.ortMs)
                        .setDecodeMs(res.decodeMs)
                        .setOcrMs(res.ocrMs)
                        .setCaptureTsMs(captureTsMs)
                        .setInferTsMs(inferTs)
                        .setEp(res.ep == null ? "" : res.ep)
                        .setInputSize(res.inputSize);
        if (res.dets != null) {
            for (ai.flow.adas.vision.TrafficYoloRunner.Det d : res.dets) {
                TrafficVisionOuter.TrafficLightColor tfl =
                        TrafficVisionOuter.TrafficLightColor.forNumber(d.tflColor);
                if (tfl == null) {
                    tfl = TrafficVisionOuter.TrafficLightColor.TFL_UNKNOWN;
                }
                TrafficVisionOuter.TrafficDetection.Builder db =
                        TrafficVisionOuter.TrafficDetection.newBuilder()
                                .setLabel(d.label == null ? "" : d.label)
                                .setConf(d.conf)
                                .setX1(d.x1)
                                .setY1(d.y1)
                                .setX2(d.x2)
                                .setY2(d.y2)
                                .setSpeedLimitKmh(d.speedLimitKmh)
                                .setTflColor(tfl)
                                .setSpeedFromOcr(d.speedFromOcr);
                b.addDets(db);
            }
        }
        return Messages.ZMQMessage.newBuilder()
                .setTimestamp(inferTs)
                .setTopic("vision/traffic_dets")
                .setTrafficDetections(b.build())
                .build();
    }

    private static ModelLongOuter.LeadTrack toLeadTrack(ai.flow.adas.vision.ModelLongParse.Lead lead) {
        ModelLongOuter.LeadTrack.Builder t = ModelLongOuter.LeadTrack.newBuilder()
                .setProb(lead.prob)
                .setProbTime(lead.probTime)
                .setDRel(lead.x[0])
                .setYRel(lead.y[0])
                .setVLead(lead.v[0])
                .setALead(lead.a[0]);
        for (int i = 0; i < lead.x.length; i++) {
            t.addX(lead.x[i]);
            t.addY(lead.y[i]);
            t.addV(lead.v[i]);
            t.addA(lead.a[i]);
        }
        return t.build();
    }
}
