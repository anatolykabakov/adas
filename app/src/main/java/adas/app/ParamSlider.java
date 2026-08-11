package adas.app;

import android.view.View;
import android.widget.SeekBar;
import android.widget.TextView;

/** Bind a SeekBar row to a float range with live label updates. */
final class ParamSlider {
    interface Listener {
        void onChanged(float value);
    }

    private final TextView labelView;
    private final TextView valueView;
    private final SeekBar seek;
    private final String name;
    private final String unit;
    private final float min;
    private final float max;
    private final int decimals;
    private Listener listener;
    private boolean suppress;

    ParamSlider(View row, String name, String unit, float min, float max, int decimals) {
        this.labelView = row.findViewById(R.id.paramLabel);
        this.valueView = row.findViewById(R.id.paramValue);
        this.seek = row.findViewById(R.id.paramSeek);
        this.name = name;
        this.unit = unit == null ? "" : unit;
        this.min = min;
        this.max = max;
        this.decimals = Math.max(0, decimals);
        this.labelView.setText(name);
        this.seek.setMax(1000);
        this.seek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                float v = progressToValue(progress);
                valueView.setText(format(v));
                if (fromUser && !suppress && listener != null) {
                    listener.onChanged(v);
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
    }

    void setListener(Listener listener) {
        this.listener = listener;
    }

    void setValue(float value) {
        suppress = true;
        float clamped = Math.max(min, Math.min(max, value));
        seek.setProgress(valueToProgress(clamped));
        valueView.setText(format(clamped));
        suppress = false;
    }

    float getValue() {
        return progressToValue(seek.getProgress());
    }

    void setEnabled(boolean enabled) {
        seek.setEnabled(enabled);
        labelView.setEnabled(enabled);
        valueView.setEnabled(enabled);
        float alpha = enabled ? 1f : 0.4f;
        labelView.setAlpha(alpha);
        valueView.setAlpha(alpha);
        seek.setAlpha(alpha);
    }

    private float progressToValue(int progress) {
        return min + (max - min) * (progress / 1000f);
    }

    private int valueToProgress(float value) {
        if (max <= min) {
            return 0;
        }
        return Math.round(1000f * (value - min) / (max - min));
    }

    private String format(float v) {
        String fmt = "%." + decimals + "f";
        if (unit.isEmpty()) {
            return String.format(fmt, v);
        }
        return String.format(fmt, v) + unit;
    }
}
