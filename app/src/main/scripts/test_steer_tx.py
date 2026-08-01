#!/usr/bin/env python3
"""
Inject SteerCommand over controls/steer; PandaService / CarController owns HCA+LDW TX.

Prereqs (on car with app + panda running):
  1. Ignition on → panda safety volkswagen @15
  2. Stock ACC engaged (Set) → controls_allowed=true
  3. Moving (not standstill) and EPS_HCA_Status ready(3)/active(5)
  4. Free port 5564: bind controls/steer PUB here

From host PC:
  adb reverse tcp:5564 tcp:5564
  adb forward tcp:5565 tcp:5565
  adb forward tcp:5566 tcp:5566

  python3 test_steer_tx.py --left --torque 220 --seconds 4

Success uses peak |Δangle| during the pulse (not angle after ramp-down).
Safety: wheels clear, hand near wheel, start with low torque. Ctrl+C → zero.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import zmq

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR / "proto"))

import messages_pb2  # noqa: E402

HZ = 50
SAFETY_VOLKSWAGEN = 15
SAFETY_ALLOUTPUT = 17


def make_steer_cmd(torque_cnm: int, enabled: bool) -> bytes:
    msg = messages_pb2.ZMQMessage()
    msg.timestamp = int(time.time() * 1_000_000)
    msg.topic = "controls/steer"
    cmd = msg.steer_command
    cmd.torque_cnm = int(torque_cnm)
    cmd.enabled = bool(enabled)
    return msg.SerializeToString()


def ramp_torque(target: int, step: int = 4) -> list[int]:
    """Respect panda max_rate_up ≈ 4 cNm per 50Hz tick."""
    out: list[int] = []
    cur = 0
    sign = 1 if target >= 0 else -1
    target_abs = abs(target)
    while cur < target_abs:
        cur = min(target_abs, cur + step)
        out.append(sign * cur)
    return out


def safety_name(mode: int) -> str:
    return {
        0: "SILENT",
        3: "ELM327",
        15: "VOLKSWAGEN",
        17: "ALLOUTPUT",
        19: "NOOUTPUT",
    }.get(mode, str(mode))


def main() -> int:
    p = argparse.ArgumentParser(
        description="Verify steering via SteerCommand → CarController HCA"
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--left", action="store_true", help="Negative torque (left)")
    g.add_argument("--right", action="store_true", help="Positive torque (right)")
    g.add_argument("--zero", action="store_true", help="Send zero / disabled only")
    p.add_argument(
        "--torque", type=int, default=50, help="Torque cNm, max 300 (default 50)"
    )
    p.add_argument(
        "--seconds",
        type=float,
        default=2.0,
        help="Hold at target (or max with --delta-deg)",
    )
    p.add_argument(
        "--delta-deg",
        type=float,
        default=None,
        help="Stop early when peak |Δ| reaches this many degrees",
    )
    p.add_argument("--tx", default="tcp://127.0.0.1:5564", help="controls/steer PUB bind")
    p.add_argument("--vehicle", default="tcp://127.0.0.1:5566", help="vehicle/state SUB")
    p.add_argument("--health", default="tcp://127.0.0.1:5565", help="panda/health SUB")
    p.add_argument("--dry-run", action="store_true", help="Print ramp only, no ZMQ")
    args = p.parse_args()

    if args.left:
        target = -abs(args.torque)
        want_delta = -abs(args.delta_deg) if args.delta_deg is not None else None
    elif args.right:
        target = abs(args.torque)
        want_delta = abs(args.delta_deg) if args.delta_deg is not None else None
    else:
        target = 0
        want_delta = None

    if args.dry_run:
        for tq in ramp_torque(target) or [0]:
            print(f"tq={tq:+4d} enabled={tq != 0}")
        return 0

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(args.tx)
    print(f"PUB controls/steer bound {args.tx} (wait for phone SUB…)")
    time.sleep(0.8)

    sub = ctx.socket(zmq.SUB)
    sub.connect(args.vehicle)
    sub.connect(args.health)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 50)

    angle0 = None
    angle = None
    peak_delta = 0.0  # signed extreme in command direction during pulse
    peak_abs = 0.0
    health = {
        "controls_allowed": None,
        "safety_mode": None,
        "tx_blocked": None,
        "ignition": None,
    }

    def poll_telemetry() -> None:
        nonlocal angle0, angle, peak_delta, peak_abs
        try:
            while True:
                raw = sub.recv(zmq.NOBLOCK)
                msg = messages_pb2.ZMQMessage()
                msg.ParseFromString(raw)
                if msg.HasField("car_state"):
                    angle = msg.car_state.steering_angle_deg
                    if angle0 is None:
                        angle0 = angle
                    elif angle is not None:
                        d = angle - angle0
                        if abs(d) > peak_abs:
                            peak_abs = abs(d)
                            peak_delta = d
                elif msg.HasField("panda_health"):
                    h = msg.panda_health
                    health["controls_allowed"] = h.controls_allowed
                    health["safety_mode"] = h.safety_mode
                    health["tx_blocked"] = h.tx_blocked
                    health["ignition"] = h.ignition_line or h.ignition_can
        except zmq.Again:
            pass

    def send_tq(tq: int, enabled: bool) -> None:
        pub.send(make_steer_cmd(tq, enabled))

    def reached_delta() -> bool:
        if want_delta is None:
            return False
        if want_delta < 0:
            return peak_delta <= want_delta
        return peak_delta >= want_delta

    print(
        "Engage stock ACC (Set) before expecting motion. "
        "Watching vehicle/state + panda/health…"
    )
    t_end_wait = time.time() + 2.0
    while time.time() < t_end_wait:
        poll_telemetry()
        time.sleep(0.05)

    print(
        f"before: angle={angle}°  controls_allowed={health['controls_allowed']}  "
        f"safety={safety_name(health['safety_mode'] or -1)}  "
        f"ignition={health['ignition']}  tx_blocked={health['tx_blocked']}"
    )
    if want_delta is not None:
        print(
            f"target: peak Δ={want_delta:+.1f}° with tq={target:+d} cNm, max {args.seconds}s"
        )
    if health["safety_mode"] not in (SAFETY_VOLKSWAGEN, SAFETY_ALLOUTPUT):
        print(
            f"WARN: safety_mode={health['safety_mode']} "
            f"(want VOLKSWAGEN={SAFETY_VOLKSWAGEN} or ALLOUTPUT={SAFETY_ALLOUTPUT})"
        )
    if health["controls_allowed"] is False and health["safety_mode"] == SAFETY_VOLKSWAGEN:
        print("WARN: controls_allowed=false — engage ACC or panda will block HCA torque")

    # Reset peak baseline after warmup
    if angle is not None:
        angle0 = angle
        peak_delta = 0.0
        peak_abs = 0.0

    dt = 1.0 / HZ
    hit_target = False
    try:
        for tq in ramp_torque(target):
            send_tq(tq, True)
            poll_telemetry()
            if reached_delta():
                hit_target = True
                break
            time.sleep(dt)

        if not hit_target:
            t_end = time.time() + args.seconds
            tick = 0
            while time.time() < t_end:
                send_tq(target, target != 0)
                poll_telemetry()
                tick += 1
                if tick % HZ == 0:
                    cur = None if angle is None or angle0 is None else angle - angle0
                    print(
                        f"  tq={target:+d} angle={angle}° Δ={cur}° peak={peak_delta:+.1f}°  "
                        f"allowed={health['controls_allowed']} blocked={health['tx_blocked']}"
                    )
                if reached_delta():
                    hit_target = True
                    print(
                        f"hit peak Δ={peak_delta:+.1f}° (want {want_delta:+.1f}°) — stopping"
                    )
                    break
                time.sleep(dt)

        for tq in reversed(ramp_torque(target)):
            send_tq(tq, True)
            poll_telemetry()
            time.sleep(dt)
        for _ in range(HZ):
            send_tq(0, False)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nabort → zero torque")
    finally:
        for _ in range(HZ):
            send_tq(0, False)
            time.sleep(dt)
        poll_telemetry()
        pub.close(0)
        sub.close(0)
        ctx.term()

    final_delta = None if angle is None or angle0 is None else angle - angle0
    print(
        f"after:  angle0={angle0}° angle={angle}° finalΔ={final_delta}° peakΔ={peak_delta:+.1f}°  "
        f"allowed={health['controls_allowed']} blocked={health['tx_blocked']}"
    )
    ok_thresh = abs(want_delta) * 0.7 if want_delta is not None else 5.0
    if peak_abs >= ok_thresh:
        print(f"OK: peak |Δ|={peak_abs:.1f}° ≥ {ok_thresh:.1f}° — HCA accepted by EPS")
        return 0
    print(
        "NO clear angle change. Check: ACC engaged, safety VOLKSWAGEN, "
        "tx_blocked, ignition, harness, and that nothing else owns :5564"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
