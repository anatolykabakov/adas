# Glossary

Terms the chapters use as if known. Each is defined where it first does real work; this is the one place
to look them all up.

## The car and the bus

* **MQB** — Volkswagen's platform (the Golf 7 here). Fixes wheelbase, steering ratio, CAN layout.
* **CAN** — the car's internal message bus; every sensor and actuator frame rides it.
* **DBC** — the text database that says which bits of which CAN frame are which physical signal.
* **panda** — comma.ai's USB CAN interface between the phone and the car, with a safety model on board.
* **EPS** — electric power steering: the rack motor that actually turns the wheels.
* **HCA / HCA_01** — the CAN frame (address `0x126`) that asks the EPS for assist torque.
* **cNm** — centinewton-metre, 1/100 N·m; the unit of the torque command. ±300 cNm = ±3 N·m.

## Geometry and control

* **SWA** — steering-wheel angle, what the column reports on CAN.
* **δ (delta)** — road-wheel angle. SWA = `steer_sign · δ · steer_ratio`.
* **steer_ratio** — degrees at the steering wheel per degree at the road wheel (~15.7 here).
* **κ (kappa)** — path curvature, 1/radius. The planner outputs this, never an angle.
* **CTE** — cross-track error: lateral offset of the car from the path centre.
* **epsi** — heading error: the angle between the car's nose and the path tangent.
* **PID** — controller summing proportional, integral and feedforward terms of an error (see
  [Angle control](../Control/AngleControl.md)).
* **understeer / tire_stiffness_factor** — the tyres turn less than geometry predicts; the factor scales
  the reference stiffness (see [Vehicle model](../Control/VehicleModel.md)).

## Localization

* **ENU** — East-North-Up, the local metric frame GPS is projected into.
* **RPY** — roll, pitch, yaw: the camera's mounting angles (also the phone-IMU mount seed).
* **EKF** — extended Kalman filter: predict-then-update estimator (see
  [Localization](../Localization/Overview.md)).
* **paramsd** — the online estimator of vehicle parameters (stiffness, ratio, biases); built and shipped
  disabled.

## Vision and runtime

* **Supercombo** — comma's driving network: from camera frames to lane lines and a plan.
* **thneed** — a recorded GPU run of Supercombo, replayed on the phone's GPU (see
  [Supercombo](../Vision/Supercombo.md)).
* **ONNX / ONNX Runtime** — the portable model format and its CPU/NNAPI runtime, the fallback path.
* **ZMQ** — the message library bridging the Java sensor side to the native C++ side.
* **protobuf** — the binary format every logged message and bag record is encoded in.

## The lineage

* **comma / comma-two** — comma.ai and its comma-two device; the openpilot lineage this stack forks.
* **openpilot / flowpilot** — comma's driving stack, and its Android port; `fp` is named after flowpilot.
* **`pp` / `fp` / `vp`** — the lateral strategies: pure pursuit, flowpilot MPC (road default), and the
  deleted VisionPilot MPC (`vp`, taught only as a toy).
* **acados** — the embedded optimal-control solver `fp` can use for its MPC.
