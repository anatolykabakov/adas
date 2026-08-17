package adas.app.record;

import adas.app.TimeUtil;
import android.media.MediaRecorder;
import android.util.Log;

import java.io.File;

/**
 * Microphone audio recorded alongside the bag, for the duration of the bag. The bag records what the
 * car did; the audio records what was happening around it — a horn, a thump, or the driver saying
 * what they just felt, timestamped better than the memory of it.
 *
 * <p>AAC in MP4, 44.1 kHz mono, 64 kbit/s: about 28 MB per hour against 1.2 GB for the video stream
 * over the same time, encoded by the hardware codec.
 *
 * <p>The start time is in the file name, {@code audio_<t>.m4a}, taken from {@link TimeUtil#nowMs} —
 * the same monotonic clock every bag message is stamped with, so aligning sound to a message is a
 * subtraction. A sidecar file holding that stamp would be one more thing to fall out of sync.
 *
 * <p>A microphone failure must never take the data recording down: no permission, a busy microphone
 * or a codec refusal all log a warning and leave the bag running without sound.
 */
public final class AudioRecorder {
    private static final String TAG = "AudioRecorder";

    private static final int SAMPLE_RATE_HZ = 44100;
    private static final int BIT_RATE = 64000;

    private MediaRecorder recorder;
    private File output;

    public synchronized boolean isRecording() {
        return recorder != null;
    }

    /** @return the output file, or null when there will be no audio; the bag runs on regardless. */
    public synchronized File start(File dir, long startMs) {
        if (recorder != null) {
            Log.i(TAG, "already recording " + output);
            return output;
        }
        File file = new File(dir, "audio_" + startMs + ".m4a");
        MediaRecorder mr = new MediaRecorder();
        try {
            mr.setAudioSource(MediaRecorder.AudioSource.MIC);
            mr.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
            mr.setAudioEncoder(MediaRecorder.AudioEncoder.AAC);
            mr.setAudioChannels(1);
            mr.setAudioSamplingRate(SAMPLE_RATE_HZ);
            mr.setAudioEncodingBitRate(BIT_RATE);
            mr.setOutputFile(file.getAbsolutePath());
            mr.prepare();
            mr.start();
        } catch (Throwable t) {
            // Usually a missing RECORD_AUDIO grant, otherwise the microphone is busy with a call.
            Log.e(TAG, "audio not recording: " + t.getMessage(), t);
            try {
                mr.release();
            } catch (Throwable ignored) {
            }
            return null;
        }
        recorder = mr;
        output = file;
        Log.i(TAG, "recording audio to " + file.getName());
        return file;
    }

    public synchronized void stop() {
        MediaRecorder mr = recorder;
        recorder = null;
        if (mr == null) {
            return;
        }
        try {
            mr.stop();
        } catch (Throwable t) {
            // stop() throws when less than a frame was captured, leaving a broken file. Removing it
            // is right here: an empty m4a reads as lost audio rather than as nothing to record.
            Log.w(TAG, "stop failed, file is probably empty: " + t.getMessage());
            if (output != null && output.exists() && output.length() < 1024) {
                //noinspection ResultOfMethodCallIgnored
                output.delete();
            }
        } finally {
            try {
                mr.release();
            } catch (Throwable ignored) {
            }
        }
        Log.i(TAG, "audio stopped: " + (output != null ? output.getName() : "null"));
        output = null;
    }
}
