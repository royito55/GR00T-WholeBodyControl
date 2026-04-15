# decoupled_wbc Overview

This repository implements a whole-body control (WBC) stack for humanoid loco-manipulation with primary support for Unitree G1. It provides:
- A real-time control loop that runs on ROS 2, receives high-level commands, and sends joint targets to the robot or simulator.
- A teleoperation stack that turns human inputs (Pico, Vive, Leap, etc.) into upper-body targets and navigation commands.
- A lower-body policy that uses ONNX models (OpenGearWbc) for locomotion.
- Data collection utilities that export trajectories to the LeRobot dataset format.

## High-level Architecture

### Runtime Data Flow (Typical)
1. Teleop or inference publishes a control goal on the ROS topic `ControlPolicy/upper_body_pose`.
2. The control loop subscribes to that topic, combines upper-body goals with the lower-body policy, and sends joint targets to the robot or sim.
3. Observations, actions, and metadata are published for data collection.

### Major Components
- Control loop: runs the main G1 policy and publishes telemetry and exported data.
- Policies: combine upper-body interpolation/IK with lower-body ONNX locomotion.
- Robot model: kinematics, joint groups, limits, and joint mappings.
- Environment: binds the robot to either simulator or hardware and enforces safety checks.
- Teleop: streamers + IK retargeting to produce upper-body targets.
- Data exporter: writes trajectories as a LeRobot dataset.
- Deployment: tmux-based orchestration for control, teleop, camera, and data collection.

## How This Fits With Isaac GR00T / VLA Checkpoints

This repo does not contain a direct Isaac GR00T runtime. Instead, it is a controller and teleop stack that expects a stream of targets on a ROS topic. A VLA checkpoint can be integrated by adding a policy or service that:
- Consumes observations (images, proprio, task context).
- Produces commands compatible with the WBC interface:
  - `target_upper_body_pose` (joint positions for the upper body)
  - `wrist_pose` (optional end-effector pose)
  - `navigate_cmd` (x, y, yaw)
  - `base_height_command`
- Publishes these commands to `ControlPolicy/upper_body_pose`.

There is also a ZMQ-based inference server/client utility that can be used to host a model outside the control loop and query it from a policy.

### Practical Integration Path for a GR00T VLA Checkpoint
1. Run the control loop (simulation or real). This is the hard real-time loop.
2. Implement a small policy process that:
   - Loads your VLA checkpoint and runs inference.
   - Translates model output to the WBC command schema.
   - Publishes to `ControlPolicy/upper_body_pose` at ~20 Hz.
3. Optional: integrate the model server using the ZMQ inference helpers and keep the control loop unchanged.

## Key Files (Most Important)

- Control loop entry point: decoupled_wbc/control/main/teleop/run_g1_control_loop.py
- Teleop loop entry point: decoupled_wbc/control/main/teleop/run_teleop_policy_loop.py
- WBC policy wiring: decoupled_wbc/control/policy/wbc_policy_factory.py
- Whole-body policy (upper + lower): decoupled_wbc/control/policy/g1_decoupled_whole_body_policy.py
- Lower-body ONNX policy: decoupled_wbc/control/policy/g1_gear_wbc_policy.py
- G1 environment & safety: decoupled_wbc/control/envs/g1/g1_env.py
- Teleop policy (IK + streamers): decoupled_wbc/control/policy/teleop_policy.py
- Teleop device streamers: decoupled_wbc/control/teleop/teleop_streamer.py
- Data export to LeRobot: decoupled_wbc/data/exporter.py
- Deployment orchestration: scripts/deploy_g1.py
- Inference helpers (ZMQ): decoupled_wbc/control/utils/service.py

## Notes for Running a Trained VLA Checkpoint
- The WBC control loop expects upper-body joint targets; if your model outputs end-effector poses, use IK (similar to TeleopPolicy) to convert them.
- If your model outputs action trajectories, align them with the control loop frequency (default 50 Hz).
- If you plan to use camera inputs, the composed camera client + sensor server are already part of the deployment flow.
