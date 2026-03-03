# Closed-loop Bridge + WBC (Simple)

This file explains the diagram in the simplest way.

## What runs

- `run_groot_closed_loop_bridge.py`
  - Reads robot state + camera.
  - Calls the GR00T policy server.
  - Publishes a WBC goal on `/ControlPolicy/upper_body_pose`.

- `run_g1_control_loop.py`
  - Subscribes to `/ControlPolicy/upper_body_pose`.
  - Runs the WBC policy stack.
  - Sends robot commands.

## Why there are 3 policies

Inside the WBC stack:

- `G1DecoupledWholeBodyPolicy`
  - Coordinator/orchestrator.
  - Calls the upper and lower sub-policies.
  - Merges into one full-body joint target `q`.

- `InterpolationPolicy` (upper body)
  - Smoothly interpolates arm/upper-body targets over time.

- `G1GearWbcPolicy` (lower body)
  - Computes lower-body locomotion/stability action.

## Important point about `rt/lowcmd`

Only one program path publishes `rt/lowcmd` in this setup:

`run_g1_control_loop.py` -> `G1Env.queue_action(...)` -> `BodyCommandSender.send_command(...)` -> `rt/lowcmd`

So:

- Multiple policies compute action parts.
- A single control-loop path publishes the final low-level command.

## One-line summary

Bridge publishes goals; WBC merges upper+lower policy outputs; one final sender publishes `rt/lowcmd`.
