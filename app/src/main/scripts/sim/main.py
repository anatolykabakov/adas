#!/usr/bin/env python3
"""Interactive MetaDrive run: camera, overlay, live controller.

For scored controller runs use ``sim.eval`` — this one is for looking at the thing drive.

Controllers (all are the real C++ services via ``pyadas``, parameters from config.json):
  - fp           : the shipped lateral MPC (default on the phone)
  - mpc          : VisionPilot spatial MPC
  - pure_pursuit : Pure Pursuit + angle PID
  - straight     : no lateral control, baseline

Lane source:
  - gt         : MetaDrive ground-truth lane boundaries
  - supercombo : live supercombo.onnx lanes (+ plan) — the full perception chain

Usage:
  cd app/src/main/scripts && python3 -m sim.main --controller fp --lanes supercombo --show
  python3 -m sim.main --controller fp --lanes gt --overlay --out-dir run_gt --max-frames 300
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any, Dict, Optional

import _path  # noqa: F401

import cv2
import numpy as np

try:
    from metadrive.component.sensors.rgb_camera import RGBCamera
    from metadrive.envs.metadrive_env import MetaDriveEnv
    from metadrive.utils import setup_logger
except ImportError as e:
    print(f"Error importing MetaDrive: {e}")
    print("Install MetaDrive simulator:")
    print("  pip uninstall metadrive")
    print("  pip install metadrive-simulator")
    print(
        "  # or: git clone https://github.com/metadriverse/metadrive && pip install -e ."
    )
    sys.exit(1)

from core.frames import (
    DEFAULT_MAX_STEER_DEG,
    DRAW_Y_SIGN,
    METADRIVE_STEER_FROM_DEVICE,
    PP_Y_SIGN,
)
from core.lane_keep import (
    LaneKeepController,
    LaneKeepResult,
    build_centerline_polyline,
    load_vehicle_config,
)
from core.lane_keep_viz import draw_lane_keep_overlay
from core.path_fusion import iso_left_polyline_to_device, path_from_supercombo
from core.phone_rt import PhoneRtGeometry
from core.supercombo_compare import (
    DEFAULT_MODEL,
    SupercomboBev,
    draw_supercombo_overlay,
    make_overlay_geometry,
    resolve_supercombo_model,
    supercombo_lanes_to_ego,
)
from core.supercombo_parse import SupercomboOut
from sim.camera_utils import (
    CameraGeometry as MetaDriveCamGeom,
    CameraParams,
    METADRIVE_MOUNT_HEIGHT_M,
    METADRIVE_MOUNT_PITCH_DEG,
)
from sim.eval import MAX_STEERING_DEG, centerline_ahead
from sim.observation_parser import ObservationParser
from sim.track import TRACKS, env_config, resolve


def save_data(img, obs_data, frame, out_dir: Path, overlay_img=None):
    prefix = out_dir / f"frame_{frame:05d}"
    np.savetxt(f"{prefix}_left_line_road.txt", obs_data["lanes"]["left_road"])
    np.savetxt(f"{prefix}_right_line_road.txt", obs_data["lanes"]["right_road"])
    np.savetxt(f"{prefix}_left_line_world.txt", obs_data["lanes"]["left_world"])
    np.savetxt(f"{prefix}_right_line_world.txt", obs_data["lanes"]["right_world"])

    img = (overlay_img if overlay_img is not None else img).copy()
    if img.dtype != np.uint8:
        img = (np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8)
    cv2.imwrite(f"{prefix}.jpg", img)


class MetaDriveSimulator:
    """MetaDrive loop with lane-keeping controller + optional logging."""

    def __init__(self, args):
        setup_logger(True)
        self.args = args
        self.out_dir = Path(args.out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        # Same track and same step as sim.eval, so what you watch here is what the scored run
        # measures; --track random falls back to MetaDrive's own random map.
        self.track = None if args.track == "random" else resolve(args.track)
        if self.track is not None:
            config = env_config(self.track, args.seed)
            config["traffic_density"] = args.traffic_density
            if args.speed is None:
                args.speed = self.track.speed_mps
        else:
            config = {
                "num_scenarios": args.num_scenarios,
                "start_seed": args.seed,
                "traffic_density": args.traffic_density,
                # 9 × 10 ms = 90 ms per control step — the phone's median vision period.
                "decision_repeat": 9,
                "physics_world_step_size": 0.01,
                "horizon": 1_000_000,
            }
        if args.speed is None:
            args.speed = 12.0

        vehicle_config = {"image_source": "rgb"}
        if abs(args.offset) > 1e-6:
            vehicle_config.update(
                spawn_lateral=float(args.offset),
                spawn_velocity=[float(args.speed), 0.0],
                spawn_velocity_car_frame=True,
            )
            config["out_of_road_done"] = False
            config["on_continuous_line_done"] = False

        config.update(
            {
                "use_render": not args.no_render,
                "manual_control": False,
                "random_traffic": args.traffic_density > 0,
                "image_observation": True,
                "show_terrain": True,
                "log_level": 50,
                "sensors": dict(rgb=[RGBCamera, args.width, args.height]),
                "vehicle_config": vehicle_config,
            }
        )

        self.env = MetaDriveEnv(config)
        self.env.reset()
        self.road_network = self.env.current_map.road_network
        self.dt_s = float(config["physics_world_step_size"] * config["decision_repeat"])
        self._cte_log: list[float] = []

        self.camera_params = CameraParams(self.env)
        self._sync_calib_from_mount()
        self.camera_geometry = MetaDriveCamGeom(
            self.camera_params.intrinsics,
            self.camera_params.extrinsics,
        )
        np.savetxt(self.out_dir / "intrinsics.txt", self.camera_params.intrinsics)
        np.savetxt(self.out_dir / "extrinsics.txt", self.camera_params.extrinsics)

        self.observation_parser = ObservationParser(self.env)
        self.observation_parser.set_camera_mount(
            self.camera_params.mount_offset,
            self.camera_params.mount_hpr,
        )
        veh_cfg = dict(load_vehicle_config()) if args.config else {}
        if veh_cfg and not args.vehicle_model:
            # MetaDrive's ego is kinematic; the Golf understeer model would over-turn every
            # arc here (sim.vehicle_calib).
            veh_cfg["lat_use_vehicle_model"] = False
        self.controller = LaneKeepController(
            mode=args.controller,
            desired_speed=args.speed,
            max_steer_deg=args.max_steer_deg,
            wheelbase=args.wheelbase,
            pp_k_dd=args.pp_k_dd,
            pp_ld_min=args.pp_ld_min,
            pp_ld_max=args.pp_ld_max,
            pp_shift=args.pp_shift,
            dt_s=self.dt_s,
            vehicle_config=veh_cfg or None,
        )

        self.supercombo: Optional[SupercomboBev] = None
        if args.lanes == "supercombo" or args.draw_supercombo:
            model = resolve_supercombo_model(args.supercombo_model)
            self.supercombo = SupercomboBev(model)
            K = self.camera_params.intrinsics
            self.supercombo.set_calib(
                roll_deg=args.roll_deg,
                pitch_deg=args.pitch_deg,
                yaw_deg=args.yaw_deg,
                fx=float(K[0, 0]),
                fy=float(K[1, 1]),
                cx=float(K[0, 2]),
                cy=float(K[1, 2]),
                use_warp=True,
            )
            if not self.supercombo._ensure():
                print(f"WARNING: supercombo unavailable: {self.supercombo.error}")
            else:
                print(f"Supercombo → {self.supercombo.model_path} (calib warp on)")

        self.frame = 0
        self._last_sc: Optional[SupercomboOut] = None
        self._log_writer = None
        self._log_file = None
        self._log_path: Path | None = None
        if args.log_csv:
            log_path = Path(args.log_csv)
            if not log_path.is_absolute():
                log_path = self.out_dir / log_path
            self._log_path = log_path
            self._log_file = log_path.open("w", newline="")
            self._log_writer = csv.DictWriter(
                self._log_file,
                fieldnames=[
                    "frame",
                    "mode",
                    "lane_source",
                    "speed_mps",
                    "steer_rad",
                    "steer_norm",
                    "throttle",
                    "brake",
                    "e_y",
                    "e_psi_rad",
                    "curvature",
                    "lookahead_m",
                    "target_x",
                    "target_y",
                    "status",
                    "pos_x",
                    "pos_y",
                    "heading_deg",
                ],
            )
            self._log_writer.writeheader()

        K = self.camera_params.intrinsics
        print(
            f"Controller={args.controller}  lanes={args.lanes}  speed={args.speed} m/s  "
            f"traffic_density={args.traffic_density}  "
            f"mount h={args.camera_height:.2f}m pitch={args.pitch_deg:.3f}° "
            f"K=({K[0,0]:.1f},{K[1,1]:.1f}) @{int(args.width)}x{int(args.height)}  "
            f"show={args.show}  save_every={args.save_every}  overlay={args.overlay}"
        )
        print(f"Output → {self.out_dir.resolve()}")
        if args.show:
            cv2.namedWindow("MetaDrive | camera + PP", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("MetaDrive | camera + PP", args.width, args.height)

    def _sync_calib_from_mount(self) -> None:
        """Fill None CLI calib from live MetaDrive mount (not phone 1.1 / prior 1.40)."""
        args = self.args
        aad = self.camera_params.aad_overlay_params()
        self._mount_aad = aad
        if args.camera_height is None:
            args.camera_height = float(aad["camera_height"])
        if args.pitch_deg is None:
            args.pitch_deg = float(aad["pitch_deg"])
        if args.yaw_deg is None:
            args.yaw_deg = float(aad["yaw_deg"])
        if args.roll_deg is None:
            args.roll_deg = float(aad["roll_deg"])
        print(
            f"Calib from MetaDrive mount: h={args.camera_height:.3f}m "
            f"cam_x={aad['cam_x']:.3f} pitch={args.pitch_deg:.3f}° "
            f"yaw={args.yaw_deg:.3f}° roll={args.roll_deg:.3f}° "
            f"draw={args.draw} "
            f"(CLI override keeps explicit flags)"
        )

    def _make_overlay_geom(
        self, fx: float, fy: float, cx: float, cy: float, w: int, h: int
    ):
        """Phone-Rt (matches warp) or AAD (with mount cam_x)."""
        args = self.args
        if args.draw == "phone-rt":
            return PhoneRtGeometry(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                roll_deg=args.roll_deg,
                pitch_deg=args.pitch_deg,
                yaw_deg=args.yaw_deg,
                width=w,
                height=h,
                camera_height_m=args.camera_height,
            )
        aad = getattr(self, "_mount_aad", None) or self.camera_params.aad_overlay_params()
        return make_overlay_geometry(
            fx,
            fy,
            cx,
            cy,
            w,
            h,
            camera_height=args.camera_height,
            pitch_deg=args.pitch_deg,
            yaw_deg=args.yaw_deg,
            roll_deg=args.roll_deg,
            cam_x=float(aad.get("cam_x", 0.0)),
            cam_y_left=float(aad.get("cam_y_left", 0.0)),
        )

    def _overlay_y_sign(self) -> float:
        # Phone-Rt expects device Y-right; AAD expects ISO via DRAW_Y_SIGN.
        return PP_Y_SIGN if self.args.draw == "phone-rt" else DRAW_Y_SIGN

    def _to_uint8(self, img: np.ndarray) -> np.ndarray:
        if img.dtype == np.uint8:
            return img
        return (np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8)

    def _run_supercombo(self, camera_image: np.ndarray) -> Optional[SupercomboOut]:
        if self.supercombo is None:
            return None
        bgr = self._to_uint8(camera_image)
        # MetaDrive RGB camera is often RGB float; convert to BGR for OpenCV/ONNX path
        if camera_image.ndim == 3 and camera_image.shape[2] == 3:
            # perceive() returns RGB; OpenCV expects BGR
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
        out = self.supercombo.infer(bgr, cache_key=self.frame)
        self._last_sc = out
        return out

    def _control_lanes(
        self,
        gt_lanes: Dict[str, Any],
        sc: Optional[SupercomboOut],
    ) -> Dict[str, Any]:
        """Lane dict for overlay / GT mid. Supercombo control uses path_fusion."""
        if self.args.lanes != "supercombo" or sc is None:
            return gt_lanes
        sc_lanes = supercombo_lanes_to_ego(
            sc,
            min_lane_prob=self.args.min_lane_prob,
            y_sign=PP_Y_SIGN,
        )
        if len(sc_lanes["left_road"]) < 2 or len(sc_lanes["right_road"]) < 2:
            return gt_lanes
        return {
            **gt_lanes,
            "left_road": sc_lanes["left_road"],
            "right_road": sc_lanes["right_road"],
        }

    def _compute_control(
        self,
        speed: float,
        lanes: Dict[str, Any],
        sc: Optional[SupercomboOut],
    ) -> tuple[Optional[LaneKeepResult], list[float]]:
        poly = None
        if self.args.lanes == "supercombo":
            # Any controller, not just pure_pursuit: gating this on the mode meant `fp` silently
            # drove on ground truth while the window claimed to be running the model.
            if sc is not None:
                if self.args.pp_on == "plan":
                    # Same fusion as Android TopicConvert.laneLinesToPath
                    poly = path_from_supercombo(sc, min_lane_prob=self.args.min_lane_prob)
                else:
                    poly = build_centerline_polyline(
                        lanes.get("left_road"), lanes.get("right_road")
                    )
        else:
            # Same reference sim.eval scores against: lane centre from the map, ISO left+ → device.
            poly_iso = centerline_ahead(self.env.agent, self.road_network)
            poly = iso_left_polyline_to_device(poly_iso) if poly_iso.size else None

        lk = self.controller.compute_from_polyline(
            speed,
            poly,
            yaw_rate=float(getattr(self.env.agent, "yaw_rate", 0.0)),
            lane_anchored=poly is not None and self.args.lanes == "gt",
        )
        # Device-frame δ → MetaDrive actuator (left+). The actuator is a fraction of the car's
        # own max steering, not of the controller's clamp: normalising by the latter used to
        # multiply an 8°-clamped PP command by five.
        steer_act = METADRIVE_STEER_FROM_DEVICE * float(
            np.clip(np.rad2deg(lk.steer_rad) / MAX_STEERING_DEG, -1.0, 1.0)
        )
        return lk, [steer_act, lk.throttle, lk.brake]

    def _make_viz_image(
        self,
        camera_image: np.ndarray,
        lanes: dict,
        lk: LaneKeepResult | None,
        sc: Optional[SupercomboOut],
        gt_lanes: Optional[dict] = None,
    ) -> np.ndarray:
        need_viz = self.args.show or self.args.overlay
        if not need_viz:
            return self._to_uint8(camera_image)

        bgr = self._to_uint8(camera_image)
        if camera_image.ndim == 3 and camera_image.shape[2] == 3:
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)

        K = self.camera_params.intrinsics
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        h, w = bgr.shape[:2]
        geom = self._make_overlay_geom(fx, fy, cx, cy, w, h)
        y_sign = self._overlay_y_sign()

        if (
            self.args.draw_supercombo or self.args.lanes == "supercombo"
        ) and sc is not None:
            draw_supercombo_overlay(
                bgr,
                sc,
                geom,
                w,
                h,
                y_sign=y_sign,
                min_lane_prob=self.args.min_lane_prob,
            )
        elif sc is None and self.supercombo is not None and self.supercombo.error:
            cv2.putText(
                bgr,
                f"supercombo ERR: {self.supercombo.error[:48]}",
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

        if lk is not None:
            # PP polyline is device Y-right; GT lanes for draw → convert to device.
            viz_lanes = None
            if self.args.draw_gt_lanes or self.args.lanes == "gt":
                # Always the MetaDrive boundaries here, and those are ISO left+ whichever
                # source drives control, so the conversion to device is unconditional.
                src = gt_lanes if gt_lanes is not None else lanes
                viz_lanes = {}
                for key in ("left_road", "right_road"):
                    arr = src.get(key)
                    if arr is None:
                        continue
                    viz_lanes[key] = iso_left_polyline_to_device(
                        np.asarray(arr, dtype=np.float64)
                    )
            bgr = draw_lane_keep_overlay(
                bgr,
                lk,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                w=w,
                h=h,
                lanes=viz_lanes,
                pitch_deg=self.args.pitch_deg,
                yaw_deg=self.args.yaw_deg,
                roll_deg=self.args.roll_deg,
                camera_height=self.args.camera_height,
                waypoint_shift=self.args.pp_shift,
                draw_bev=self.args.bev,
                draw_footer=True,
                geom=geom,
                y_sign=y_sign,
            )
            cv2.putText(
                bgr,
                f"lanes={self.args.lanes}",
                (8, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )
        return bgr

    def _log_step(self, odometry, lk_result):
        if self._log_writer is None:
            return
        pp = lk_result.pure_pursuit if lk_result else None
        tgt_x = tgt_y = ""
        la = ""
        if pp is not None:
            la = f"{pp.lookahead_m:.3f}"
            if pp.target_ego is not None:
                tgt_x = f"{pp.target_ego[0]:.3f}"
                tgt_y = f"{pp.target_ego[1]:.3f}"
        self._log_writer.writerow(
            {
                "frame": self.frame,
                "mode": lk_result.mode if lk_result else self.args.controller,
                "lane_source": self.args.lanes,
                "speed_mps": f"{odometry['speed']:.3f}",
                "steer_rad": f"{lk_result.steer_rad:.5f}" if lk_result else "0",
                "steer_norm": f"{lk_result.steer_norm:.5f}" if lk_result else "0",
                "throttle": f"{lk_result.throttle:.3f}" if lk_result else "",
                "brake": f"{lk_result.brake:.3f}" if lk_result else "",
                "e_y": f"{lk_result.e_y:.4f}" if lk_result else "",
                "e_psi_rad": f"{lk_result.e_psi:.5f}" if lk_result else "",
                "curvature": f"{lk_result.curvature:.6f}" if lk_result else "",
                "lookahead_m": la,
                "target_x": tgt_x,
                "target_y": tgt_y,
                "status": lk_result.status if lk_result else "",
                "pos_x": f"{odometry['position'][0]:.2f}",
                "pos_y": f"{odometry['position'][1]:.2f}",
                "heading_deg": f"{np.rad2deg(odometry['heading']):.1f}",
            }
        )

    def run(self):
        try:
            while True:
                obs_data = self.observation_parser.make_perception_data()
                odometry = obs_data["odometry"]
                gt_lanes = obs_data["lanes"]
                camera_image = obs_data["camera_image"]

                sc = None
                if camera_image is not None and self.supercombo is not None:
                    sc = self._run_supercombo(camera_image)

                lanes = self._control_lanes(gt_lanes, sc)
                lk, action = self._compute_control(odometry["speed"], lanes, sc)

                self._log_step(odometry, lk)

                if camera_image is not None:
                    viz_img = self._make_viz_image(
                        camera_image, lanes, lk, sc, gt_lanes=gt_lanes
                    )

                    if self.args.show:
                        cv2.imshow("MetaDrive | camera + PP", viz_img)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            break

                    if self.frame % self.args.save_every == 0:
                        save_data(
                            camera_image,
                            obs_data,
                            self.frame,
                            self.out_dir,
                            overlay_img=viz_img if self.args.overlay else None,
                        )

                gt_cte = float("nan")
                try:
                    _, lat = self.env.agent.lane.local_coordinates(
                        self.env.agent.position
                    )
                    gt_cte = float(lat)  # right-positive, same convention as sim.eval
                    self._cte_log.append(gt_cte)
                except Exception:
                    pass

                if self.frame % self.args.print_every == 0:
                    steer_deg = np.rad2deg(lk.steer_rad) if lk else 0.0
                    extra = ""
                    if lk and lk.pure_pursuit is not None:
                        extra = f" Ld={lk.pure_pursuit.lookahead_m:.1f}m"
                    sc_tag = ""
                    if sc is not None:
                        probs = ",".join(f"{l.prob:.2f}" for l in sc.lanes)
                        sc_tag = f" sc_p=[{probs}]"
                    lane = self.env.agent.lane
                    radius = float(getattr(lane, "radius", 0.0) or 0.0)
                    where = "straight" if radius <= 0 else f"arc R={radius:.0f}m"
                    print(
                        f"frame {self.frame}: {where} v={odometry['speed']:.1f} m/s "
                        f"CTE={gt_cte:+.2f} m δ={steer_deg:+.1f}°{extra} "
                        f"lanes={self.args.lanes}"
                        f"{sc_tag}"
                    )

                self.env.step(action)
                self.frame += 1
                if self.args.max_frames and self.frame >= self.args.max_frames:
                    break

        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        finally:
            self.cleanup()

    def cleanup(self):
        if self._cte_log:
            cte = np.abs(np.array(self._cte_log))
            print(
                f"summary: |CTE| median {np.median(cte):.2f} m / p95 {np.percentile(cte, 95):.2f} / "
                f"max {cte.max():.2f} over {len(cte)} frames "
                f"(per-segment metrics — python3 -m sim.eval --track {self.args.track})"
            )
        if self._log_file is not None:
            self._log_file.close()
            if self._log_path is not None:
                print(f"Log → {self._log_path.resolve()}")
        if self.args.show:
            cv2.destroyAllWindows()
        print("Closing environment...")
        self.env.close()
        print("Done.")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--no-render", action="store_true", help="Disable MetaDrive 3D window")
    p.add_argument(
        "--show",
        action="store_true",
        help="Live OpenCV window: camera + lane/PP overlay (q or Esc to quit)",
    )
    p.add_argument(
        "--controller",
        choices=LaneKeepController.MODES,
        default="pure_pursuit",
        help="Lane keeping algorithm",
    )
    p.add_argument(
        "--lanes",
        choices=("gt", "supercombo"),
        default="supercombo",
        help="Lane source for control: MetaDrive GT or live supercombo.onnx",
    )
    p.add_argument(
        "--pp-on",
        choices=("lanes", "plan"),
        default="plan",
        help="When --lanes supercombo: PP on model plan (default) or lane centerline",
    )
    p.add_argument(
        "--supercombo-model",
        type=str,
        default=str(DEFAULT_MODEL),
        help="Path to supercombo.onnx",
    )
    p.add_argument("--min-lane-prob", type=float, default=0.3)
    p.add_argument(
        "--draw-supercombo",
        action="store_true",
        default=True,
        help="Draw supercombo plan/lanes/edges on camera (default on)",
    )
    p.add_argument("--no-draw-supercombo", action="store_false", dest="draw_supercombo")
    p.add_argument(
        "--draw-gt-lanes",
        action="store_true",
        help="Also draw MetaDrive GT lane boundaries on overlay",
    )
    p.add_argument("--out-dir", default="run", help="Output directory for frames/logs")
    p.add_argument(
        "--log-csv", default="lane_keep.csv", help="CSV log filename (in out-dir)"
    )
    p.add_argument("--save-every", type=int, default=30, help="Save frame every N steps")
    p.add_argument(
        "--print-every", type=int, default=30, help="Print status every N steps"
    )
    p.add_argument(
        "--overlay", action="store_true", help="Draw lane/PP overlay on saved frames"
    )
    p.add_argument(
        "--bev", action="store_true", default=True, help="BEV inset on overlay"
    )
    p.add_argument("--no-bev", action="store_false", dest="bev")

    p.add_argument(
        "--track",
        default="highway",
        choices=(*TRACKS, "random"),
        help="track as in sim.eval (default highway); random = random MetaDrive map",
    )
    p.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="start offset from center, m (+ right), at target speed immediately",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=None,
        help="target speed m/s (default from track)",
    )
    p.add_argument("--max-steer-deg", type=float, default=DEFAULT_MAX_STEER_DEG)
    p.add_argument("--wheelbase", type=float, default=2.636)

    # Pure pursuit (interactive_visualizer defaults)
    p.add_argument("--pp-k-dd", type=float, default=0.4)
    p.add_argument("--pp-ld-min", type=float, default=3.0)
    p.add_argument("--pp-ld-max", type=float, default=20.0)
    p.add_argument("--pp-shift", type=float, default=1.40)

    # Overlay / warp calib — default None → filled from MetaDrive mount after env reset
    # (height≈1.5 m, pitch≈0.6° look-up). Do not use phone mount 1.1 / prior 1.40.
    p.add_argument(
        "--pitch-deg",
        type=float,
        default=None,
        help=f"Warp/overlay pitch deg (default: MetaDrive mount ≈{METADRIVE_MOUNT_PITCH_DEG})",
    )
    p.add_argument(
        "--yaw-deg",
        type=float,
        default=None,
        help="Warp/overlay yaw deg (default: MetaDrive mount)",
    )
    p.add_argument(
        "--roll-deg",
        type=float,
        default=None,
        help="Warp/overlay roll deg (default: MetaDrive mount)",
    )
    p.add_argument(
        "--camera-height",
        type=float,
        default=None,
        help=f"AAD overlay camera height m (default: MetaDrive mount ≈{METADRIVE_MOUNT_HEIGHT_M})",
    )
    p.add_argument(
        "--draw",
        choices=("phone-rt", "aad"),
        default="phone-rt",
        help="Overlay projector: phone-rt=V·R·V⁻¹ (matches warp, default); aad=CameraGeometry+height/cam_x",
    )

    p.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after N frames (0 = no limit)",
    )
    p.add_argument(
        "--config",
        action="store_true",
        default=True,
        help="load controller params from assets/config.json (default yes)",
    )
    p.add_argument("--no-config", action="store_false", dest="config")
    p.add_argument(
        "--vehicle-model",
        action="store_true",
        help="keep Golf understeer model (simulator default is kinematic)",
    )
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--num-scenarios", type=int, default=100)
    p.add_argument(
        "--traffic-density",
        type=float,
        default=0.0,
        help="Background traffic density (0 = no other vehicles)",
    )
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    return p


def main():
    args = build_arg_parser().parse_args()
    try:
        MetaDriveSimulator(args).run()
    except Exception as e:
        print(f"Error running simulation: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
