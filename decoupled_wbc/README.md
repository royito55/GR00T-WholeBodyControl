# decoupled_wbc Real Robot Notes

This README documents the changes made to get `control/main/teleop/run_g1_control_loop.py` working on the real G1 after the original `ChannelFactoryInitialize` / ROS 2 collision and the later no-motion issue.

## Problem Summary

Initial failure:
- `ChannelFactoryInitialize(...)` failed with a CycloneDDS domain error.
- The SDK low-level example worked, so network access to the robot was fine.

After that was fixed:
- `rt/lowstate` was received.
- `rt/lowcmd` was published.
- The robot still did not move.

The final working path required three categories of fixes:
1. Separate ROS DDS from Unitree DDS by process.
2. Add a 23-DoF hardware compatibility config for a G1 that physically lacks some 29-DoF joints.
3. Sync outgoing `low_cmd.mode_machine` to the live value reported by `rt/lowstate`.

## Changes Made

### 1. ROS / Unitree DDS Process Split

Reason:
- ROS 2 (`rclpy`) and Unitree SDK both create DDS participants.
- Running both in the same Python process caused the original `ChannelFactoryInitialize` failure.

Changes:
- Added [`control/utils/ros_process_bridge.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/utils/ros_process_bridge.py)
- Updated [`control/main/teleop/run_g1_control_loop.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/run_g1_control_loop.py)

Behavior now:
- Parent process owns Unitree DDS only.
- Spawned subprocess owns ROS pub/sub/service only.
- IPC uses `multiprocessing.Queue`.

ROS bridge responsibilities:
- publish env state
- publish lower-body policy status
- publish joint safety status
- provide robot config service
- forward keyboard topics
- subscribe to upper-body control goals

### 2. Remove Unused ROS Node From `G1Env`

Reason:
- `G1Env` was also creating a ROS node in the Unitree process.
- That was an unnecessary second DDS collision point.

Changes:
- Updated [`control/envs/g1/g1_env.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/g1_env.py)

Behavior now:
- `G1Env` initializes Unitree channels only.
- No ROS node is created inside `G1Env`.

### 3. Lowstate / Lowcmd Debug Instrumentation

Reason:
- After the DDS/process fix, the controller ran but the robot still did not move.
- Needed proof that sensing and command publishing were alive before debugging semantics.

Changes:
- Updated [`control/envs/g1/utils/state_processor.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/utils/state_processor.py)
- Updated [`control/envs/g1/utils/command_sender.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/utils/command_sender.py)

Debug behavior:
- `BodyStateProcessor` logs:
  - missing `rt/lowstate`
  - missing `rt/secondary_imu`
  - first received lowstate packet
  - periodic alive logs
- `BodyCommandSender` logs:
  - periodic outgoing `lowcmd`
  - `mode_machine`
  - `mode_pr`
  - first motor mode / q / dq / gains

Useful signal from debugging:
- incoming `lowstate.mode_machine` was `4`
- config had hard-coded `MODE_MACHINE: 5`

### 4. Sync `mode_machine` From Live `rt/lowstate`

Reason:
- The working SDK example mirrors `mode_machine` from live lowstate.
- This repo had been hard-coding `MODE_MACHINE` from YAML.
- That mismatch likely caused the robot to ignore commands even though `lowcmd` was arriving.

Changes:
- Updated [`control/envs/g1/utils/command_sender.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/utils/command_sender.py)

Behavior now:
- `BodyCommandSender` subscribes to `rt/lowstate`
- if a live packet is available, `self.low_cmd.mode_machine` is overwritten from `latest_low_state.mode_machine`
- debug print shows the actual outgoing `mode_machine`

### 5. 23-DoF Hardware Compatibility Config

Reason:
- The repo is structurally 29-DoF:
  - robot model is 29-DoF
  - config is 29-DoF
  - message layout is 29-slot HG low-level command/state
- Actual hardware appears to be a 23-DoF G1.
- The practical compatibility approach is to keep the 29-slot layout but zero gains for joints that do not exist.

Changes:
- Added [`control/main/teleop/configs/g1_23dof_compat_gear_wbc.yaml`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/configs/g1_23dof_compat_gear_wbc.yaml)
- Updated [`control/main/teleop/configs/configs.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/configs/configs.py)

New CLI flag:
- `--robot-variant g1_23dof_compat`

Compatibility YAML behavior:
- keeps `ROBOT_TYPE: g1_29dof` so the HG message layout stays compatible with the rest of the stack
- zeros PD gains for these six joints:
  - waist roll
  - waist pitch
  - left wrist pitch
  - left wrist yaw
  - right wrist pitch
  - right wrist yaw

This is a compatibility mode, not a native 23-DoF robot model.

## Working Command

Use this on the real robot:

```bash
python control/main/teleop/run_g1_control_loop.py --interface real --robot-variant g1_23dof_compat --no-with-hands
```

If the interface alias does not resolve correctly on a given machine, use the direct NIC name:

```bash
python control/main/teleop/run_g1_control_loop.py --interface enP8p1s0 --robot-variant g1_23dof_compat --no-with-hands
```

## Expected Debug Output

Healthy receive path:
- `[BodyStateProcessor] First lowstate packet received: ...`
- periodic `[BodyStateProcessor] lowstate stream alive: ...`

Healthy command path:
- `[BodyCommandSender] lowcmd publish: ...`

Important field to check:
- outgoing `mode_machine` should match the incoming lowstate machine mode

## Remaining Caveats

This is not a full native 23-DoF port.

Still 29-DoF in core modeling:
- [`control/robot_model/instantiation/g1.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/robot_model/instantiation/g1.py) still instantiates a 29-DoF URDF
- [`control/robot_model/supplemental_info/g1/g1_supplemental_info.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/robot_model/supplemental_info/g1/g1_supplemental_info.py) still assumes 29 body DoFs

So this setup is best understood as:
- 29-slot policy/model path
- 23-DoF hardware compatibility at the low-level actuator side

If a cleaner long-term solution is needed, the next step would be:
1. add a real `g1_23dof` robot model / supplemental info path
2. verify the lower-body policy dimensions against that model
3. decide whether to keep the 29-slot annealed policy mask or retrain / swap models

## Files Changed

Main runtime changes:
- [`control/main/teleop/run_g1_control_loop.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/run_g1_control_loop.py)
- [`control/utils/ros_process_bridge.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/utils/ros_process_bridge.py)
- [`control/envs/g1/g1_env.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/g1_env.py)

Debug / low-level command changes:
- [`control/envs/g1/utils/state_processor.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/utils/state_processor.py)
- [`control/envs/g1/utils/command_sender.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/envs/g1/utils/command_sender.py)

Config changes:
- [`control/main/teleop/configs/configs.py`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/configs/configs.py)
- [`control/main/teleop/configs/g1_23dof_compat_gear_wbc.yaml`](/home/unitree/git-repo/GR00T-WholeBodyControl/decoupled_wbc/control/main/teleop/configs/g1_23dof_compat_gear_wbc.yaml)
