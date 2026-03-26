import os
import multiprocessing as mp
import queue
import time
from typing import Any, Optional

from decoupled_wbc.control.main.constants import (
    CONTROL_GOAL_TOPIC,
    JOINT_SAFETY_STATUS_TOPIC,
    KEYBOARD_INPUT_TOPIC,
    LOWER_BODY_POLICY_STATUS_TOPIC,
    ROBOT_CONFIG_TOPIC,
    STATE_TOPIC_NAME,
)
from decoupled_wbc.control.utils.keyboard_dispatcher import KEYBOARD_LISTENER_TOPIC_NAME

ONE_SHOT_CONTROL_GOAL_KEYS = (
    "toggle_policy_action",
    "toggle_activation",
    "toggle_data_collection",
    "toggle_data_abort",
    "reset_env_and_policy",
)


def _drain_latest(mp_queue, value):
    try:
        while True:
            mp_queue.get_nowait()
    except queue.Empty:
        pass
    mp_queue.put(value)


def _drain_latest_control_goal(mp_queue, value):
    latest = None
    try:
        while True:
            latest = mp_queue.get_nowait()
    except queue.Empty:
        pass

    if isinstance(latest, dict) and isinstance(value, dict):
        merged = value.copy()
        for key in ONE_SHOT_CONTROL_GOAL_KEYS:
            merged[key] = bool(latest.get(key, False) or value.get(key, False))
        value = merged

    mp_queue.put(value)


def _ros_bridge_worker(
    node_name: str,
    robot_config: dict,
    command_queue,
    control_goal_queue,
    keyboard_queue,
    stop_event,
    zmq_control_goal_host: Optional[str] = None,
    zmq_control_goal_port: int = 5556,
):
    debug_zmq = os.getenv("GR00T_DEBUG_ZMQ", "").lower() in {"1", "true", "yes"}
    debug_zmq_verbose = os.getenv("GR00T_DEBUG_ZMQ_VERBOSE", "").lower() in {"1", "true", "yes"}
    last_debug_log_time = 0.0
    last_zmq_msg_time = None

    from std_msgs.msg import String as RosStringMsg

    from decoupled_wbc.control.utils.ros_utils import (
        ROSManager,
        ROSMsgPublisher,
        ROSMsgSubscriber,
        ROSServiceServer,
    )

    ros_manager = ROSManager(node_name=node_name)
    node = ros_manager.node

    ROSServiceServer(ROBOT_CONFIG_TOPIC, robot_config)
    publishers = {
        'state': ROSMsgPublisher(STATE_TOPIC_NAME),
        'lower_body_policy_status': ROSMsgPublisher(LOWER_BODY_POLICY_STATUS_TOPIC),
        'joint_safety_status': ROSMsgPublisher(JOINT_SAFETY_STATUS_TOPIC),
    }
    keyboard_listener_pub = node.create_publisher(RosStringMsg, KEYBOARD_LISTENER_TOPIC_NAME, 1)

    if zmq_control_goal_host:
        import msgpack
        import msgpack_numpy as mnp
        import zmq

        _zmq_context = zmq.Context()
        _zmq_sub = _zmq_context.socket(zmq.SUB)
        _zmq_sub.setsockopt(zmq.SUBSCRIBE, b"")
        _zmq_sub.setsockopt(zmq.RCVTIMEO, 10)
        _zmq_sub.setsockopt(zmq.LINGER, 0)
        _zmq_sub.connect(f"tcp://{zmq_control_goal_host}:{zmq_control_goal_port}")
        print(f"Receiving control goals via ZMQ from tcp://{zmq_control_goal_host}:{zmq_control_goal_port}")
        control_goal_subscriber = None
    else:
        _zmq_context = None
        _zmq_sub = None
        control_goal_subscriber = ROSMsgSubscriber(CONTROL_GOAL_TOPIC)

    def keyboard_input_callback(msg: RosStringMsg):
        _drain_latest(keyboard_queue, msg.data)

    keyboard_subscription = node.create_subscription(
        RosStringMsg, KEYBOARD_INPUT_TOPIC, keyboard_input_callback, 10
    )

    try:
        while ros_manager.ok() and not stop_event.is_set():
            if _zmq_sub is not None:
                try:
                    payload = _zmq_sub.recv()
                    upper_body_cmd = msgpack.unpackb(payload, object_hook=mnp.decode)
                    _drain_latest_control_goal(control_goal_queue, upper_body_cmd)
                    last_zmq_msg_time = time.monotonic()
                    if debug_zmq:
                        toggle_data_collection = upper_body_cmd.get("toggle_data_collection")
                        toggle_data_abort = upper_body_cmd.get("toggle_data_abort")
                        if toggle_data_collection or toggle_data_abort:
                            print(
                                "[ZMQ RECORD] "
                                f"toggle_data_collection={bool(toggle_data_collection)} "
                                f"toggle_data_abort={bool(toggle_data_abort)}"
                            )
                    if debug_zmq_verbose:
                        now = time.monotonic()
                        if now - last_debug_log_time >= 1.0:
                            last_debug_log_time = now
                            msg_ts = upper_body_cmd.get("timestamp")
                            age_ms = None if msg_ts is None else (time.monotonic() - msg_ts) * 1000.0
                            navigate_cmd = upper_body_cmd.get("navigate_cmd")
                            base_height = upper_body_cmd.get("base_height_command")
                            wrist_pose = upper_body_cmd.get("wrist_pose")
                            target_upper = upper_body_cmd.get("target_upper_body_pose")
                            toggle_data_collection = upper_body_cmd.get("toggle_data_collection")
                            toggle_data_abort = upper_body_cmd.get("toggle_data_abort")
                            age_str = "unknown" if age_ms is None else f"{age_ms:.1f}"
                            print(
                                "[ZMQ DEBUG VERBOSE] "
                                f"recv age_ms={age_str} "
                                f"navigate_cmd={navigate_cmd} "
                                f"base_height={base_height} "
                                f"toggle_data_collection={toggle_data_collection} "
                                f"toggle_data_abort={toggle_data_abort} "
                                f"wrist_pose_len={0 if wrist_pose is None else len(wrist_pose)} "
                                f"target_upper_len={0 if target_upper is None else len(target_upper)}"
                            )
                except zmq.Again:
                    if debug_zmq_verbose and last_zmq_msg_time is None:
                        now = time.monotonic()
                        if now - last_debug_log_time >= 1.0:
                            last_debug_log_time = now
                            print("[ZMQ DEBUG VERBOSE] waiting for first control goal over ZMQ")
            else:
                upper_body_cmd = control_goal_subscriber.get_msg()
                if upper_body_cmd is not None:
                    _drain_latest_control_goal(control_goal_queue, upper_body_cmd)

            try:
                command, payload = command_queue.get(timeout=0.01)
            except queue.Empty:
                continue

            if command == 'shutdown':
                break
            if command == 'keyboard_listener_key':
                keyboard_listener_pub.publish(RosStringMsg(data=payload))
                continue

            publisher = publishers.get(command)
            if publisher is None:
                print(f'Unknown ROS bridge command: {command}')
                continue
            publisher.publish(payload)
    finally:
        try:
            node.destroy_subscription(keyboard_subscription)
        except Exception:
            pass
        if _zmq_sub is not None:
            _zmq_sub.close()
            _zmq_context.term()
        ros_manager.shutdown()


class ROSProcessBridge:
    def __init__(
        self,
        node_name: str,
        robot_config: dict,
        start_method: str = 'spawn',
        zmq_control_goal_host: Optional[str] = None,
        zmq_control_goal_port: int = 5556,
    ):
        self.node_name = node_name
        self.robot_config = robot_config
        self.zmq_control_goal_host = zmq_control_goal_host
        self.zmq_control_goal_port = zmq_control_goal_port
        self.mp_context = mp.get_context(start_method)
        self.command_queue = self.mp_context.Queue()
        self.control_goal_queue = self.mp_context.Queue(maxsize=1)
        self.keyboard_queue = self.mp_context.Queue(maxsize=1)
        self.stop_event = self.mp_context.Event()
        self.process = None

    def start(self):
        if self.process is not None and self.process.is_alive():
            return
        self.stop_event.clear()
        self.process = self.mp_context.Process(
            target=_ros_bridge_worker,
            args=(
                self.node_name,
                self.robot_config,
                self.command_queue,
                self.control_goal_queue,
                self.keyboard_queue,
                self.stop_event,
                self.zmq_control_goal_host,
                self.zmq_control_goal_port,
            ),
            daemon=True,
        )
        self.process.start()

    def is_alive(self) -> bool:
        return self.process is not None and self.process.is_alive()

    def _publish(self, channel: str, payload: Any):
        self.command_queue.put((channel, payload))

    def publish_state(self, payload: dict):
        self._publish('state', payload)

    def publish_lower_body_policy_status(self, payload: dict):
        self._publish('lower_body_policy_status', payload)

    def publish_joint_safety_status(self, payload: dict):
        self._publish('joint_safety_status', payload)

    def publish_keyboard_listener_key(self, key: str):
        self._publish('keyboard_listener_key', key)

    def get_latest_control_goal(self) -> Optional[dict]:
        latest = None
        try:
            while True:
                latest = self.control_goal_queue.get_nowait()
        except queue.Empty:
            return latest

    def get_latest_keyboard_key(self) -> Optional[str]:
        latest = None
        try:
            while True:
                latest = self.keyboard_queue.get_nowait()
        except queue.Empty:
            return latest

    def sleep_to_rate(self, loop_start_time: float, frequency: float):
        remaining = (1.0 / frequency) - (time.monotonic() - loop_start_time)
        if remaining > 0:
            time.sleep(remaining)

    def close(self):
        self.stop_event.set()
        self.command_queue.put(('shutdown', None))
        if self.process is not None and self.process.is_alive():
            self.process.join(timeout=5)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
