#!/usr/bin/env python3
"""ADAS bag → CSV for PlotJuggler (Android session layout).

One row per message with hierarchical columns (vehicle/…, sensors/imu/…,
panda/…, gps/…, vision/…, control/lane_keep/…, controls/steer/…,
localization/…, middleware/stats/…).

Latency (same as latency.py) — look under ``latency/`` in PlotJuggler:
  latency/vision/infer_ms              OrtSession.run only
  latency/vision/prep_ms               e2e − infer  (warp + pack)
  latency/vision/e2e_ms                infer_ts − capture
  latency/lane_keep/capture_to_publish_ms
  latency/lane_keep/vision_to_publish_ms
Do not use controls/steer for e2e (chassis republish with stale vision_ts).

Usage:
  python3 vis/export_to_plotjuggler.py /path/to/adas_bags/2026_07_18_09_45_15 -o /tmp/out
  # writes /tmp/out/2026_07_18_09_45_15.csv
  plotjuggler /tmp/out/2026_07_18_09_45_15.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Callable, Optional

# scripts/ is parent of vis/; _path lives there.
_SCRIPTS = Path(__file__).resolve().parent.parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import _path  # noqa: F401

from vis.android_bag_player import AndroidBagPlayer

SOURCE_ORDER = (
    "vehicle",
    "sensors",
    "panda",
    "gps",
    "vision",
    "camera",
    "can",
    "control",
    "controls",
    "localization",
    "calibration",
    "model",
    "middleware",
)


def vals(prefix: str, **fields: Any) -> dict:
    return {f"{prefix}/{k}": v for k, v in fields.items() if v is not None}


class Collector:
    def __init__(self) -> None:
        self.rows: list[dict] = []
        self._prev_wall: dict[str, int] = {}
        self._prev_data: dict[str, int] = {}
        self.counts: dict[str, int] = {}
        self.lags_ms: dict[str, list[int]] = {}
        self.zmq_intervals_ms: dict[str, list[int]] = {}
        self.intervals_ms: dict[str, list[int]] = {}

    def add(
        self,
        source: str,
        stream: str,
        prefix: str,
        wall_ms: int,
        data_ms: int,
        extra: dict,
    ) -> None:
        # X-axis uses bag/ZMQ wall time (same clock for all topics). Payload
        # clocks can diverge (e.g. old middleware steady_clock vs BOOTTIME).
        stat_key = f"{source}.{stream}"
        self.counts[stat_key] = self.counts.get(stat_key, 0) + 1

        lag_ms = wall_ms - data_ms
        self.lags_ms.setdefault(stat_key, []).append(lag_ms)

        row: dict = {
            "_wall_ms": wall_ms,
            "_prefix": prefix,
            f"{prefix}/lag_ms": lag_ms,
        }
        prev_w = self._prev_wall.get(prefix)
        prev_d = self._prev_data.get(prefix)
        if prev_w is not None:
            zmq_interval_ms = wall_ms - prev_w
            row[f"{prefix}/zmq_interval_ms"] = zmq_interval_ms
            self.zmq_intervals_ms.setdefault(stat_key, []).append(zmq_interval_ms)
        if prev_d is not None and data_ms != prev_d:
            interval_ms = data_ms - prev_d
            row[f"{prefix}/interval_ms"] = interval_ms
            self.intervals_ms.setdefault(stat_key, []).append(interval_ms)
        self._prev_wall[prefix] = wall_ms
        self._prev_data[prefix] = data_ms
        row.update(extra)
        self.rows.append(row)

    def print_summary(self) -> None:
        by_source: dict[str, list[tuple[str, int]]] = {}
        for key, count in self.counts.items():
            source, stream = key.split(".", 1)
            by_source.setdefault(source, []).append((stream, count))
        for source in SOURCE_ORDER:
            if source not in by_source:
                continue
            items = ", ".join(f"{s}:{c}" for s, c in sorted(by_source[source]))
            print(f"[{source}] {items}")
        for source in sorted(by_source):
            if source in SOURCE_ORDER:
                continue
            items = ", ".join(f"{s}:{c}" for s, c in sorted(by_source[source]))
            print(f"[{source}] {items}")
        print(f"total rows: {len(self.rows)}")

    def _stat_sort_key(self, key: str) -> tuple:
        source = key.split(".", 1)[0]
        try:
            source_idx = SOURCE_ORDER.index(source)
        except ValueError:
            source_idx = len(SOURCE_ORDER)
        return source_idx, key

    def print_stats_array(self, name: str, values: list[int]) -> None:
        if not values:
            return
        print(f"{name}:")
        print(f"  min={min(values):.1f} ms")
        print(f"  max={max(values):.1f} ms")
        print(f"  median={statistics.median(values):.1f} ms")
        print(f"  mean={statistics.mean(values):.1f} ms")
        if len(values) >= 2:
            print(f"  stdev={statistics.stdev(values):.1f} ms")

    def print_stats(self) -> None:
        keys = sorted(
            set(self.lags_ms) | set(self.zmq_intervals_ms) | set(self.intervals_ms),
            key=self._stat_sort_key,
        )
        for key in keys:
            if key in self.lags_ms:
                self.print_stats_array(f"{key} lags", self.lags_ms[key])
            if key in self.zmq_intervals_ms:
                self.print_stats_array(f"{key} zmq_intervals", self.zmq_intervals_ms[key])
            if key in self.intervals_ms:
                self.print_stats_array(f"{key} intervals", self.intervals_ms[key])

    def _column_rank(self, col: str) -> tuple:
        if col == "timestamp":
            return (0, 0, col)
        top = col.split("/", 1)[0]
        try:
            source_idx = SOURCE_ORDER.index(top)
        except ValueError:
            source_idx = len(SOURCE_ORDER)
        return (1, source_idx, col)

    def _align_outlier_clocks(self) -> None:
        """Shift topics whose timestamps don't overlap the densest stream.

        Old middleware used steady_clock while bags use BOOTTIME (~minutes offset).
        """
        from collections import defaultdict

        walls: dict[str, list[int]] = defaultdict(list)
        for r in self.rows:
            walls[str(r["_prefix"])].append(int(r["_wall_ms"]))
        if len(walls) < 2:
            return
        primary = max(walls, key=lambda p: len(walls[p]))
        p_min = min(walls[primary])
        p_max = max(walls[primary])
        for prefix, ws in walls.items():
            t_min, t_max = min(ws), max(ws)
            if t_max < p_min or t_min > p_max:
                delta = p_min - t_min
                print(
                    f"clock-align '{prefix}': +{delta} ms (no overlap with '{primary}')"
                )
                for r in self.rows:
                    if r["_prefix"] == prefix:
                        r["_wall_ms"] = int(r["_wall_ms"]) + delta

    def write_csv(self, path: Path) -> None:
        if not self.rows:
            return
        self._align_outlier_clocks()
        t0 = min(int(r["_wall_ms"]) for r in self.rows)
        for r in self.rows:
            wall = int(r.pop("_wall_ms"))
            r.pop("_prefix", None)
            r["timestamp"] = (wall - t0) / 1000.0
        self.rows.sort(key=lambda r: r["timestamp"])
        fields = ["timestamp"]
        for row in self.rows:
            fields.extend(k for k in row if k not in fields)
        fields[1:] = sorted(fields[1:], key=self._column_rank)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore", restval="")
            w.writeheader()
            w.writerows(self.rows)
        span = self.rows[-1]["timestamp"] - self.rows[0]["timestamp"]
        print(f"{path}  (t=0..{span:.1f}s, {len(self.rows)} rows, {len(fields)} cols)")
        print("Note: sparse CSV — each row is one topic; empty cells are normal.")
        print("In PlotJuggler: DataLoader → CSV, X axis = timestamp, then pick series.")


def _data_ms(msg: Any, wall_ms: int) -> int:
    ts = int(getattr(msg, "timestamp", 0) or 0)
    return ts if ts > 0 else wall_ms


def process_vehicle(m: Any, wall_ms: int) -> tuple[int, dict]:
    p = "vehicle"
    ws = m.wheel_speeds
    return _data_ms(m, wall_ms), vals(
        p,
        v_ego=m.v_ego,
        v_ego_raw=m.v_ego_raw,
        a_ego=m.a_ego,
        standstill=int(m.standstill),
        wheel_fl=ws.fl,
        wheel_fr=ws.fr,
        wheel_rl=ws.rl,
        wheel_rr=ws.rr,
        steering_angle_deg=m.steering_angle_deg,
        steering_rate_deg=m.steering_rate_deg,
        steering_torque=m.steering_torque,
        steering_pressed=int(m.steering_pressed),
        yaw_rate=m.yaw_rate,
        gas=m.gas,
        gas_pressed=int(m.gas_pressed),
        brake=m.brake,
        brake_pressed=int(m.brake_pressed),
        gear=m.gear,
        cruise_main=int(m.cruise_main_switch),
        cruise_set=int(m.cruise_set),
        cruise_resume=int(m.cruise_resume),
        cruise_cancel=int(m.cruise_cancel),
        cruise_accel=int(m.cruise_accel),
        cruise_decel=int(m.cruise_decel),
        cruise_gap=m.cruise_gap_adjust,
        acc_status=m.acc_status,
        cruise_available=int(m.cruise_available),
        cruise_engaged=int(m.cruise_engaged),
    )


def process_imu(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals(
        "sensors/imu",
        accel_x=m.accel_x,
        accel_y=m.accel_y,
        accel_z=m.accel_z,
        gyro_x=m.gyro_x,
        gyro_y=m.gyro_y,
        gyro_z=m.gyro_z,
        mag_x=m.mag_x,
        mag_y=m.mag_y,
        mag_z=m.mag_z,
        accuracy=m.accuracy,
        temperature=m.temperature,
    )


def process_panda(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals(
        "panda",
        controls_allowed=int(m.controls_allowed),
        safety_mode=m.safety_mode,
        safety_param=m.safety_param,
        voltage_mv=m.voltage_mv,
        current_ma=m.current_ma,
        tx_blocked=m.tx_blocked,
        heartbeat_lost=int(m.heartbeat_lost),
        ignition_line=int(m.ignition_line),
        ignition_can=int(m.ignition_can),
        power_save=int(m.power_save_enabled),
        alt_exp=m.alternative_experience,
        fault_status=m.fault_status,
        faults=m.faults_pkt,
    )


def process_gps_location(m: Any, wall_ms: int, origin: list) -> tuple[int, dict]:
    """origin: mutable [lat0, lon0] set on first fix."""
    lat, lon = float(m.latitude), float(m.longitude)
    if origin[0] is None:
        origin[0], origin[1] = lat, lon
    # local ENU metres (approx)
    r = 6371000.0
    dlat = math.radians(lat - origin[0])
    dlon = math.radians(lon - origin[1])
    x = dlon * math.cos(math.radians(origin[0])) * r
    y = dlat * r
    return _data_ms(m, wall_ms), vals(
        "gps/location",
        lat=lat,
        lon=lon,
        alt=m.altitude,
        speed=m.speed,
        bearing=m.bearing,
        h_acc=m.horizontal_accuracy,
        v_acc=m.vertical_accuracy,
        sats=m.satellites_used,
        hdop=m.hdop,
        vdop=m.vdop,
        fix=int(m.fix_type),
        x=x,
        y=y,
    )


def process_gps_data(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals(
        "gps/data",
        lat=m.latitude,
        lon=m.longitude,
        alt=m.altitude,
        speed=m.speed,
        bearing=m.bearing,
    )


def _lane_y_at(lane: Any, x_pts: list, x_query: float) -> Optional[float]:
    ys = list(lane.y)
    if not ys or not x_pts or len(ys) != len(x_pts):
        return None
    # nearest sample
    best_i = min(range(len(x_pts)), key=lambda i: abs(x_pts[i] - x_query))
    return float(ys[best_i])


def process_lanes(m: Any, wall_ms: int) -> tuple[int, dict]:
    x_pts = list(m.x)
    names = ("left_far", "left_near", "right_near", "right_far")
    capture = int(getattr(m, "capture_ts_ms", 0) or 0)
    infer = int(getattr(m, "infer_ts_ms", 0) or 0)
    infer_dur = float(getattr(m, "infer_duration_ms", 0) or 0)
    data_ms = _data_ms(m, wall_ms)
    publish = data_ms
    out: dict = {"vision/lanes/frame_id": m.frame_id, "vision/lanes/n_x": len(x_pts)}
    if capture > 0:
        out["vision/lanes/capture_ts_ms"] = capture
    if infer_dur > 0:
        out["vision/lanes/infer_duration_ms"] = infer_dur
        out["latency/vision/infer_ms"] = infer_dur
    if infer > 0:
        out["vision/lanes/infer_ts_ms"] = infer
        if capture > 0 and infer >= capture:
            e2e = float(infer - capture)
            # Full vision chain (prep + session.run + parse), not pure ONNX
            out["vision/lanes/latency_e2e_ms"] = e2e
            out["latency/vision/e2e_ms"] = e2e
            if infer_dur > 0 and e2e >= infer_dur:
                prep = e2e - infer_dur
                out["vision/lanes/prep_ms"] = prep
                out["latency/vision/prep_ms"] = prep
        if publish >= infer:
            out["vision/lanes/latency_post_infer_ms"] = publish - infer
    if capture > 0 and publish >= capture:
        out["vision/lanes/latency_capture_ms"] = publish - capture
    for i, name in enumerate(names):
        if i >= len(m.lanes):
            break
        lane = m.lanes[i]
        out[f"vision/lanes/{name}/prob"] = lane.prob
        for xq, tag in ((10.0, "y10"), (20.0, "y20"), (30.0, "y30")):
            y = _lane_y_at(lane, x_pts, xq)
            if y is not None:
                out[f"vision/lanes/{name}/{tag}"] = y
    # mid path from near lanes
    if len(m.lanes) >= 3:
        yl = _lane_y_at(m.lanes[1], x_pts, 20.0)
        yr = _lane_y_at(m.lanes[2], x_pts, 20.0)
        if yl is not None and yr is not None:
            out["vision/lanes/mid_y20"] = 0.5 * (yl + yr)
    return data_ms, out


def process_intrinsics(m: Any, wall_ms: int) -> tuple[int, dict]:
    fx = fy = cx = cy = None
    if len(m.intrinsic_calibration) >= 4:
        fx, fy, cx, cy = m.intrinsic_calibration[:4]
    return _data_ms(m, wall_ms), vals(
        "camera/intrinsics",
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        focal_px=m.focal_length_px,
        width=m.capture_width,
        height=m.capture_height,
    )


def process_can(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals("can/rx", n_frames=len(m.frames))


def process_lane_keep(m: Any, wall_ms: int) -> tuple[int, dict]:
    data_ms = int(getattr(m, "publish_ts_ms", 0) or 0) or _data_ms(m, wall_ms)
    capture = int(getattr(m, "capture_ts_ms", 0) or 0)
    vision = int(getattr(m, "vision_ts_ms", 0) or 0)
    chassis = int(getattr(m, "chassis_ts_ms", 0) or 0)
    publish = int(getattr(m, "publish_ts_ms", 0) or 0) or data_ms
    lat_cap = (publish - capture) if capture > 0 and publish >= capture else None
    lat_v = (publish - vision) if vision > 0 and publish >= vision else None
    lat_c = (publish - chassis) if chassis > 0 and publish >= chassis else None
    out = vals(
        "control/lane_keep",
        steer_rad=m.steer_rad,
        steer_norm=m.steer_norm,
        throttle=m.throttle,
        brake=m.brake,
        lookahead_m=m.lookahead_m,
        target_x=m.target_x,
        target_y=m.target_y,
        has_target=int(m.has_target),
        curvature=m.curvature,
        status_ok=int(m.status == "ok"),
        capture_ts_ms=capture or None,
        vision_ts_ms=vision or None,
        chassis_ts_ms=chassis or None,
        publish_ts_ms=publish or None,
        latency_capture_ms=lat_cap,
        latency_vision_ms=lat_v,
        latency_chassis_ms=lat_c,
    )
    # Same metrics as latency.py — easy PlotJuggler tree: latency/lane_keep/…
    if lat_cap is not None:
        out["latency/lane_keep/capture_to_publish_ms"] = float(lat_cap)
    if lat_v is not None:
        out["latency/lane_keep/vision_to_publish_ms"] = float(lat_v)
    return data_ms, out


def process_steer(m: Any, wall_ms: int) -> tuple[int, dict]:
    # Do NOT export capture/vision→publish latency here: controls/steer republishes
    # on chassis with a stale vision_ts (looks like ~200 ms). Use control/lane_keep.
    chassis = int(getattr(m, "chassis_ts_ms", 0) or 0)
    publish = int(getattr(m, "publish_ts_ms", 0) or 0) or wall_ms
    data_ms = chassis if chassis > 0 else publish
    return data_ms, vals(
        "controls/steer",
        torque_cnm=m.torque_cnm,
        enabled=int(m.enabled),
        chassis_ts_ms=chassis or None,
        publish_ts_ms=publish or None,
    )


def process_localization(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals(
        "localization/pose",
        x=m.x,
        y=m.y,
        yaw=m.yaw,
        v=m.v,
        yaw_rate=m.yaw_rate,
        odom_x=m.odom_x,
        odom_y=m.odom_y,
        ekf_x=m.ekf_x,
        ekf_y=m.ekf_y,
    )


def process_camera_calib(m: Any, wall_ms: int) -> tuple[int, dict]:
    return _data_ms(m, wall_ms), vals(
        "calibration/camera",
        roll_deg=m.roll_deg,
        pitch_deg=m.pitch_deg,
        yaw_deg=m.yaw_deg,
        camera_height_m=m.camera_height_m,
        fx=m.fx,
        fy=m.fy,
        cx=m.cx,
        cy=m.cy,
        calibration_success=int(m.calibration_success),
        n_updates=m.n_updates,
        vp_u=m.vp_u,
        vp_v=m.vp_v,
        has_vp=int(m.has_vp),
        cal_percent=m.cal_percent,
        cal_status=m.cal_status,
    )


def process_camera_odometry(m: Any, wall_ms: int) -> tuple[int, dict]:
    trans = list(m.trans)
    rot = list(m.rot)
    out: dict = {"model/camera_odometry/frame_id": m.frame_id}
    for i, ax in enumerate(("x", "y", "z")):
        if i < len(trans):
            out[f"model/camera_odometry/trans_{ax}"] = float(trans[i])
        if i < len(rot):
            out[f"model/camera_odometry/rot_{ax}"] = float(rot[i])
    return _data_ms(m, wall_ms), out


def _svc_key(name: str) -> str:
    return "".join(c if c.isalnum() or c in "_-" else "_" for c in (name or "unknown"))


def process_middleware_stats(m: Any, wall_ms: int) -> tuple[int, dict]:
    out = vals(
        "middleware/stats",
        dropped_total=int(m.dropped_total),
        services=int(m.services),
        running=int(m.running),
        any_lagging=int(m.any_lagging),
    )
    for svc in m.services_timing:
        key = _svc_key(svc.name)
        p = f"middleware/stats/{key}"
        out.update(
            vals(
                p,
                running=int(svc.running),
                messages_processed=int(svc.messages_processed),
                timers_fired=int(svc.timers_fired),
                exceptions=int(svc.exceptions),
                dropped=int(svc.dropped),
                inbox_depth=int(svc.inbox_depth),
                backlog_depth=int(svc.backlog_depth),
                last_cb_ms=svc.last_cb_ms,
                mean_cb_ms=svc.mean_cb_ms,
                max_cb_ms=svc.max_cb_ms,
                period_ms=svc.period_ms,
                last_dt_ms=svc.last_dt_ms,
                mean_dt_ms=svc.mean_dt_ms,
                max_dt_ms=svc.max_dt_ms,
                lagging=int(svc.lagging),
            )
        )
        for tm in getattr(svc, "timers", []) or []:
            tkey = (
                _svc_key(tm.name) if getattr(tm, "name", "") else f"{int(tm.period_ms)}ms"
            )
            tp = f"{p}/{tkey}"
            out.update(
                vals(
                    tp,
                    period_ms=tm.period_ms,
                    last_dt_ms=tm.last_dt_ms,
                    mean_dt_ms=tm.mean_dt_ms,
                    max_dt_ms=tm.max_dt_ms,
                    lagging=int(tm.lagging),
                    fired=int(tm.fired),
                )
            )
    return _data_ms(m, wall_ms), out


def process_traffic_dets(m: Any, wall_ms: int) -> tuple[int, dict]:
    """vision/traffic_dets — YOLO prep/ort/decode/ocr + e2e latency."""
    capture = int(getattr(m, "capture_ts_ms", 0) or 0)
    infer = int(getattr(m, "infer_ts_ms", 0) or 0)
    infer_dur = float(getattr(m, "infer_ms", 0) or 0)
    prep = float(getattr(m, "prep_ms", 0) or 0)
    ort = float(getattr(m, "ort_ms", 0) or 0)
    decode = float(getattr(m, "decode_ms", 0) or 0)
    ocr = float(getattr(m, "ocr_ms", 0) or 0)
    out: dict = {
        "vision/traffic_dets/frame_id": int(getattr(m, "frame_id", 0) or 0),
        "vision/traffic_dets/n_dets": len(getattr(m, "dets", []) or []),
        "vision/traffic_dets/input_size": int(getattr(m, "input_size", 0) or 0),
    }
    ep = getattr(m, "ep", "") or ""
    if ep:
        out["vision/traffic_dets/ep"] = ep
    if infer_dur > 0:
        out["vision/traffic_dets/infer_ms"] = infer_dur
        out["latency/traffic/infer_ms"] = infer_dur
    if prep > 0:
        out["vision/traffic_dets/prep_ms"] = prep
        out["latency/traffic/prep_ms"] = prep
    if ort > 0:
        out["vision/traffic_dets/ort_ms"] = ort
        out["latency/traffic/ort_ms"] = ort
    if decode > 0:
        out["vision/traffic_dets/decode_ms"] = decode
        out["latency/traffic/decode_ms"] = decode
    if ocr > 0:
        out["vision/traffic_dets/ocr_ms"] = ocr
        out["latency/traffic/ocr_ms"] = ocr
    if capture > 0:
        out["vision/traffic_dets/capture_ts_ms"] = capture
    if infer > 0:
        out["vision/traffic_dets/infer_ts_ms"] = infer
        if capture > 0 and infer >= capture:
            e2e = float(infer - capture)
            out["vision/traffic_dets/latency_e2e_ms"] = e2e
            out["latency/traffic/e2e_ms"] = e2e
    # Best speed limit among dets (for plots)
    best_lim = 0
    n_ocr = 0
    for d in getattr(m, "dets", []) or []:
        lim = int(getattr(d, "speed_limit_kmh", 0) or 0)
        if lim > best_lim:
            best_lim = lim
        if getattr(d, "speed_from_ocr", False):
            n_ocr += 1
    if best_lim > 0:
        out["vision/traffic_dets/speed_limit_kmh"] = best_lim
    out["vision/traffic_dets/n_ocr"] = n_ocr
    return _data_ms(m, wall_ms), out


def process_phone_stats(m: Any, wall_ms: int) -> tuple[int, dict]:
    """phone/stats — CPU load + thermal for correlating vision stalls."""
    out: dict = {
        "phone/stats/cpu_pct": float(getattr(m, "cpu_pct", 0) or 0),
        "phone/stats/cpu_app_pct": float(getattr(m, "cpu_app_pct", 0) or 0),
        "phone/stats/thermal_status": int(getattr(m, "thermal_status", -1)),
        "phone/stats/battery_temp_c": float(getattr(m, "battery_temp_c", 0) or 0),
        "phone/stats/battery_pct": float(getattr(m, "battery_pct", 0) or 0),
        "phone/stats/cpu_temp_c": float(getattr(m, "cpu_temp_c", 0) or 0),
        "phone/stats/skin_temp_c": float(getattr(m, "skin_temp_c", 0) or 0),
        "phone/stats/gpu_temp_c": float(getattr(m, "gpu_temp_c", 0) or 0),
        "phone/stats/cpu0_freq_khz": int(getattr(m, "cpu0_freq_khz", 0) or 0),
        "phone/stats/n_zones": len(getattr(m, "zones", []) or []),
    }
    return _data_ms(m, wall_ms), out


STREAMS: list[tuple[str, str, str, str, Callable]] = [
    # source, stream, topic, prefix, process(msg, wall_ms) -> (data_ms, extra)
    ("vehicle", "state", "vehicle/state", "vehicle", process_vehicle),
    ("sensors", "imu", "sensors/imu", "sensors/imu", process_imu),
    ("panda", "health", "panda/health", "panda", process_panda),
    ("gps", "data", "sensors/gps/data", "gps/data", process_gps_data),
    ("vision", "lanes", "vision/lanes", "vision/lanes", process_lanes),
    (
        "vision",
        "traffic_dets",
        "vision/traffic_dets",
        "vision/traffic_dets",
        process_traffic_dets,
    ),
    (
        "camera",
        "intrinsics",
        "camera/intrinsics",
        "camera/intrinsics",
        process_intrinsics,
    ),
    ("can", "rx", "can/rx", "can/rx", process_can),
    ("control", "lane_keep", "control/lane_keep", "control/lane_keep", process_lane_keep),
    ("controls", "steer", "controls/steer", "controls/steer", process_steer),
    (
        "localization",
        "pose",
        "localization/pose",
        "localization/pose",
        process_localization,
    ),
    (
        "calibration",
        "camera",
        "calibration/camera",
        "calibration/camera",
        process_camera_calib,
    ),
    (
        "model",
        "camera_odometry",
        "model/camera_odometry",
        "model/camera_odometry",
        process_camera_odometry,
    ),
    (
        "middleware",
        "stats",
        "middleware/stats",
        "middleware/stats",
        process_middleware_stats,
    ),
    ("phone", "stats", "phone/stats", "phone/stats", process_phone_stats),
]


def collect(player: AndroidBagPlayer, collector: Collector) -> None:
    gps_origin: list = [None, None]

    for source, stream, topic, prefix, process in STREAMS:
        if topic not in player.topics:
            continue
        for wall_ms, msg in player.single_type_generator_with_ts(topic):
            if msg is None:
                continue
            data_ms, extra = process(msg, wall_ms)
            collector.add(source, stream, prefix, wall_ms, data_ms, extra)

    topic = "sensors/gps/location"
    if topic in player.topics:
        for wall_ms, msg in player.single_type_generator_with_ts(topic):
            if msg is None:
                continue
            data_ms, extra = process_gps_location(msg, wall_ms, gps_origin)
            collector.add("gps", "location", "gps/location", wall_ms, data_ms, extra)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path, help="ADAS bag session dir (or .zip/.tar.gz)")
    ap.add_argument("-o", type=Path, required=True, help="output directory for CSV")
    args = ap.parse_args()

    bag = args.bag.resolve()
    if not bag.exists():
        print(f"Not found: {bag}", file=sys.stderr)
        return 1

    with AndroidBagPlayer(bag, quiet=True) as player:
        if not player.topics:
            print(f"No topics in {bag}", file=sys.stderr)
            return 1
        collector = Collector()
        collect(player, collector)
        if not collector.rows:
            print(f"No rows exported from {bag}", file=sys.stderr)
            return 1
        collector.print_summary()
        collector.print_stats()
        out = args.o.resolve() / f"{player.session_dir.name}.csv"
        collector.write_csv(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
