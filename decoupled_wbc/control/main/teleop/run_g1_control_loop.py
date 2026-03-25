from copy import deepcopy
import time

import tyro

from decoupled_wbc.control.envs.g1.g1_env import G1Env
from decoupled_wbc.control.main.teleop.configs.configs import ControlLoopConfig
from decoupled_wbc.control.main.constants import (
    DEFAULT_BASE_HEIGHT,
    DEFAULT_NAV_CMD,
    DEFAULT_WRIST_POSE,
)
from decoupled_wbc.control.policy.wbc_policy_factory import get_wbc_policy
from decoupled_wbc.control.robot_model.instantiation.g1 import (
    instantiate_g1_robot_model,
)
from decoupled_wbc.control.utils.keyboard_dispatcher import (
    KeyboardDispatcher,
    KeyboardEStop,
)
from decoupled_wbc.control.utils.ros_process_bridge import ROSProcessBridge
from decoupled_wbc.control.utils.telemetry import Telemetry

CONTROL_NODE_NAME = "ControlPolicy"


class BridgeKeyboardListenerPublisher:
    def __init__(self, ros_bridge: ROSProcessBridge):
        self.ros_bridge = ros_bridge

    def handle_keyboard_button(self, key):
        self.ros_bridge.publish_keyboard_listener_key(key)


class BridgeKeyboardDispatcher:
    def __init__(self, ros_bridge: ROSProcessBridge):
        self.ros_bridge = ros_bridge
        self.listeners = []
        self._active = False

    def register(self, listener):
        if not hasattr(listener, "handle_keyboard_button"):
            raise NotImplementedError("handle_keyboard_button is not implemented")
        self.listeners.append(listener)

    def handle_key(self, key):
        for listener in self.listeners:
            listener.handle_keyboard_button(key)

    def start(self):
        self._active = True
        print("Bridge keyboard dispatcher started")

    def stop(self):
        self._active = False

    def poll(self):
        if not self._active:
            return
        key = self.ros_bridge.get_latest_keyboard_key()
        if key is None:
            return
        self.handle_key(key)


def main(config: ControlLoopConfig):
    ros_bridge = None
    env = None
    dispatcher = None

    wbc_config = config.load_wbc_yaml()

    # Initialize telemetry
    telemetry = Telemetry(window_size=100)

    waist_location = "lower_and_upper_body" if config.enable_waist else "lower_body"
    robot_model = instantiate_g1_robot_model(
        waist_location=waist_location, high_elbow_pose=config.high_elbow_pose
    )

    ros_bridge = ROSProcessBridge(node_name=CONTROL_NODE_NAME, robot_config=config.to_dict())
    ros_bridge.start()

    env = G1Env(
        env_name=config.env_name,
        robot_model=robot_model,
        config=wbc_config,
        wbc_version=config.wbc_version,
    )
    if env.sim and not config.sim_sync_mode:
        env.start_simulator()

    wbc_policy = get_wbc_policy("g1", robot_model, wbc_config, config.upper_body_joint_speed)

    keyboard_listener_pub = BridgeKeyboardListenerPublisher(ros_bridge)
    keyboard_estop = KeyboardEStop()
    if config.keyboard_dispatcher_type == "raw":
        dispatcher = KeyboardDispatcher()
    elif config.keyboard_dispatcher_type == "ros":
        dispatcher = BridgeKeyboardDispatcher(ros_bridge)
    else:
        raise ValueError(
            f"Invalid keyboard dispatcher: {config.keyboard_dispatcher_type}, please use 'raw' or 'ros'"
        )
    dispatcher.register(env)
    dispatcher.register(wbc_policy)
    dispatcher.register(keyboard_listener_pub)
    dispatcher.register(keyboard_estop)
    dispatcher.start()

    last_teleop_cmd = None
    try:
        while True:
            t_start = time.monotonic()
            with telemetry.timer("total_loop"):
                if hasattr(dispatcher, "poll"):
                    dispatcher.poll()

                # Step simulator if in sync mode
                with telemetry.timer("step_simulator"):
                    if env.sim and config.sim_sync_mode:
                        env.step_simulator()

                # Measure observation time
                with telemetry.timer("observe"):
                    obs = env.observe()
                    wbc_policy.set_observation(obs)

                # Measure policy setup time
                with telemetry.timer("policy_setup"):
                    upper_body_cmd = ros_bridge.get_latest_control_goal()

                    t_now = time.monotonic()

                    wbc_goal = {}
                    if upper_body_cmd:
                        wbc_goal = upper_body_cmd.copy()
                        last_teleop_cmd = upper_body_cmd.copy()
                        if config.ik_indicator:
                            env.set_ik_indicator(upper_body_cmd)
                    # Send goal to policy
                    if wbc_goal:
                        wbc_goal["interpolation_garbage_collection_time"] = t_now - 2 * (
                            1 / config.control_frequency
                        )
                        wbc_policy.set_goal(wbc_goal)

                # Measure policy action calculation time
                with telemetry.timer("policy_action"):
                    wbc_action = wbc_policy.get_action(time=t_now)

                # Measure action queue time
                with telemetry.timer("queue_action"):
                    env.queue_action(wbc_action)

                # Publish status information for InteractiveModeController
                with telemetry.timer("publish_status"):
                    policy_use_action = False
                    try:
                        if hasattr(wbc_policy, "lower_body_policy"):
                            policy_use_action = getattr(
                                wbc_policy.lower_body_policy, "use_policy_action", False
                            )
                    except (AttributeError, TypeError):
                        policy_use_action = False

                    policy_status_msg = {"use_policy_action": policy_use_action, "timestamp": t_now}
                    ros_bridge.publish_lower_body_policy_status(policy_status_msg)

                    joint_safety_ok = env.get_joint_safety_status()

                    joint_safety_status_msg = {
                        "joint_safety_ok": joint_safety_ok,
                        "timestamp": t_now,
                    }
                    ros_bridge.publish_joint_safety_status(joint_safety_status_msg)

                if wbc_goal.get("toggle_data_collection", False):
                    dispatcher.handle_key("c")

                if wbc_goal.get("toggle_data_abort", False):
                    dispatcher.handle_key("x")

                if env.use_sim and wbc_goal.get("reset_env_and_policy", False):
                    print("Resetting sim environment and policy")
                    dispatcher.handle_key("k")

                    upper_body_cmd = {
                        "target_upper_body_pose": obs["q"][
                            robot_model.get_joint_group_indices("upper_body")
                        ],
                        "wrist_pose": DEFAULT_WRIST_POSE,
                        "base_height_command": DEFAULT_BASE_HEIGHT,
                        "navigate_cmd": DEFAULT_NAV_CMD,
                    }
                    last_teleop_cmd = upper_body_cmd.copy()

                    time.sleep(0.5)

                msg = deepcopy(obs)
                for key in obs.keys():
                    if key.endswith("_image"):
                        del msg[key]

                if last_teleop_cmd:
                    msg.update(
                        {
                            "action": wbc_action["q"],
                            "action.eef": last_teleop_cmd.get("wrist_pose", DEFAULT_WRIST_POSE),
                            "base_height_command": last_teleop_cmd.get(
                                "base_height_command", DEFAULT_BASE_HEIGHT
                            ),
                            "navigate_command": last_teleop_cmd.get(
                                "navigate_cmd", DEFAULT_NAV_CMD
                            ),
                            "timestamps": {
                                "main_loop": time.time(),
                                "proprio": time.time(),
                            },
                        }
                    )
                ros_bridge.publish_state(msg)
                end_time = time.monotonic()

            if not ros_bridge.is_alive():
                raise RuntimeError("ROS bridge process is not alive")

            if env.sim and (not env.sim.sim_thread or not env.sim.sim_thread.is_alive()):
                raise RuntimeError("Simulator thread is not alive")

            ros_bridge.sleep_to_rate(t_start, config.control_frequency)

            if config.verbose_timing:
                telemetry.log_timing_info(context="G1 Control Loop", threshold=0.0)
            elif (end_time - t_start) > (1 / config.control_frequency) and not config.sim_sync_mode:
                telemetry.log_timing_info(context="G1 Control Loop Missed", threshold=0.001)

    except KeyboardInterrupt:
        print("Control loop interrupted by user")
    finally:
        print("Cleaning up...")
        if dispatcher is not None:
            dispatcher.stop()
        if ros_bridge is not None:
            ros_bridge.close()
        if env is not None:
            env.close()


if __name__ == "__main__":
    config = tyro.cli(ControlLoopConfig)
    main(config)
