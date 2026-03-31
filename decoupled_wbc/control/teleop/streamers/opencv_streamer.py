import socket
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from decoupled_wbc.control.teleop.streamers.base_streamer import BaseStreamer, StreamerOutput


class OpenCVStreamer(BaseStreamer):
    """
    Streamer for receiving OpenCV hand tracking data via UDP.
    
    Expected UDP data format: 21 floats (binary float32)
    - Left wrist: [x, y, z, qx, qy, qz, qw] (7 floats)
    - Left fingers: [thumb, index, middle] (3 floats, 0=open, 1=closed)
    - Right wrist: [x, y, z, qx, qy, qz, qw] (7 floats)
    - Right fingers: [thumb, index, middle] (3 floats, 0=open, 1=closed)
    - Callback number: 1=START, 2=STOP, 3=RESET (1 float)
    """

    def __init__(self, udp_ip: str = "0.0.0.0", udp_port: int = 5005):
        super().__init__()
        self._ip = udp_ip
        self._port = udp_port
        self._sock = None
        self._socket_available = False
        self._running = False
        self._thread = None

        # Initialize with identity poses
        zeros = np.zeros(7, dtype=np.float32)
        zeros[6] = 1.0  # Set qw=1 for identity quaternion (x,y,z,qx,qy,qz,qw)
        self._latest_wrist_data = {
            "left": zeros.copy(),
            "right": zeros.copy(),
        }

        # Initialize with open fingers
        self._latest_finger_data = {
            "left": np.zeros(3, dtype=np.float32),  # [thumb, index, middle]
            "right": np.zeros(3, dtype=np.float32),
        }

        # Navigation and control state
        self.current_base_height = 0.74  # Standing height
        self.toggle_data_collection_last = False
        self.toggle_data_abort_last = False
        self.toggle_activation_last = False
        self.is_activated = False  # Track if policy has been activated

    def start_streaming(self):
        """Initialize UDP socket and start listening thread."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind((self._ip, self._port))
            self._sock.setblocking(False)
            self._socket_available = True
            print(f"[OpenCVStreamer] UDP socket bound to {self._ip}:{self._port}")
        except Exception as e:
            print(f"[OpenCVStreamer] UDP socket unavailable: {e}")
            self._socket_available = False
            return

        self._running = True
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print("[OpenCVStreamer] Listening thread started")

    def stop_streaming(self):
        """Stop the listening thread and close socket."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        print("[OpenCVStreamer] Stopped")

    def reset_status(self):
        """Reset internal state."""
        self.current_base_height = 0.74
        self.toggle_data_collection_last = False
        self.toggle_data_abort_last = False
        self.toggle_activation_last = False
        self.is_activated = False

    def _listen(self):
        """Listen for incoming UDP packets in background thread."""
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
                try:
                    arr = np.frombuffer(data, dtype=np.float32)
                    if arr.shape[0] == 21:
                        # Parse the data (copy to avoid buffer issues)
                        self._latest_wrist_data["left"] = arr[:7].copy()
                        self._latest_finger_data["left"] = arr[7:10].copy()
                        self._latest_wrist_data["right"] = arr[10:17].copy()
                        self._latest_finger_data["right"] = arr[17:20].copy()
                        callback_number = int(arr[20])

                        # Keep wrist in default orientation for testing
                        self._latest_wrist_data["left"][3] = 0.0
                        self._latest_wrist_data["left"][4] = 0.0
                        self._latest_wrist_data["left"][5] = 0.0
                        self._latest_wrist_data["left"][6] = 1.0
                        self._latest_wrist_data["right"][3] = 0.0
                        self._latest_wrist_data["right"][4] = 0.0
                        self._latest_wrist_data["right"][5] = 0.0
                        self._latest_wrist_data["right"][6] = 1.0

                        # Handle callbacks
                        if callback_number == 1:
                            print("[OpenCVStreamer] START callback received - Activating teleop")
                            self.toggle_activation_last = True
                            self.toggle_data_collection_last = True
                        elif callback_number == 2:
                            print("[OpenCVStreamer] STOP callback received")
                            self.toggle_data_abort_last = True
                        elif callback_number == 3:
                            print("[OpenCVStreamer] RESET callback received")
                            self.reset_status()
                except Exception as e:
                    print(f"[OpenCVStreamer] Error parsing data: {e}")
            except BlockingIOError:
                # No data available, continue
                time.sleep(0.001)
            except Exception as e:
                if self._running:
                    print(f"[OpenCVStreamer] Error in listen loop: {e}")

    def get(self) -> StreamerOutput:
        """
        Get current hand tracking data as StreamerOutput.
        
        Returns:
            StreamerOutput with left/right wrist poses as 4x4 matrices and finger data.
        """
        if not self._socket_available:
            # Return identity poses if socket unavailable
            return StreamerOutput(
                ik_data={
                    "left_wrist": np.eye(4),
                    "right_wrist": np.eye(4),
                    "left_fingers": {"position": self._generate_finger_data("left")},
                    "right_fingers": {"position": self._generate_finger_data("right")},
                },
                control_data={
                    "base_height_command": self.current_base_height,
                    "navigate_cmd": [0.0, 0.0, 0.0],
                },
                source="opencv",
            )

        # Convert 7D pose data to 4x4 transformation matrices
        left_wrist_T = self._pose_to_matrix(self._latest_wrist_data["left"])
        right_wrist_T = self._pose_to_matrix(self._latest_wrist_data["right"])

        # Generate finger data
        left_fingers = self._generate_finger_data("left")
        right_fingers = self._generate_finger_data("right")
        # Toggle events (edge detection)
        toggle_activation_tmp = self.toggle_activation_last
        toggle_data_collection_tmp = self.toggle_data_collection_last
        toggle_data_abort_tmp = self.toggle_data_abort_last

        toggle_activation = False
        toggle_data_collection = False
        toggle_data_abort = False

        if toggle_activation_tmp:
            toggle_activation = True
            self.toggle_activation_last = False
            self.is_activated = not self.is_activated
            print(f"[OpenCVStreamer] Teleop policy {'ACTIVATED' if self.is_activated else 'DEACTIVATED'}")

        if toggle_data_collection_tmp:
            toggle_data_collection = True
            self.toggle_data_collection_last = False

        if toggle_data_abort_tmp:
            toggle_data_abort = True
            self.toggle_data_abort_last = False

        return StreamerOutput(
            ik_data={
                "left_wrist": left_wrist_T,
                "right_wrist": right_wrist_T,
                "left_fingers": {"position": left_fingers},
                "right_fingers": {"position": right_fingers},
            },
            control_data={
                "base_height_command": self.current_base_height,
                "navigate_cmd": [0.0, 0.0, 0.0],  # No navigation from OpenCV
            },
            teleop_data={
                "toggle_activation": toggle_activation,  # Send the toggle EVENT, not the state
            },
            data_collection_data={
                "toggle_data_collection": toggle_data_collection,
                "toggle_data_abort": toggle_data_abort,
            },
            source="opencv",
        )

    def _pose_to_matrix(self, pose_7d: np.ndarray) -> np.ndarray:
        """
        Convert 7D pose [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix.
        
        Args:
            pose_7d: Array of [x, y, z, qx, qy, qz, qw]
            
        Returns:
            4x4 transformation matrix
        """
        if pose_7d.shape[0] != 7:
            print(f"[OpenCVStreamer] Invalid pose shape: {pose_7d.shape}, returning identity")
            return np.eye(4)

        xyz = pose_7d[:3]
        quat_xyzw = pose_7d[3:]  # [qx, qy, qz, qw]

        # Handle all-zero quaternion case
        if np.allclose(quat_xyzw, 0):
            quat_xyzw = np.array([0, 0, 0, 1], dtype=np.float32)

        # Convert quaternion to rotation matrix
        # scipy expects [x, y, z, w] format
        rotation_matrix = R.from_quat(quat_xyzw).as_matrix()

        # Build 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = xyz

        return T

    def _generate_finger_data(self, hand: str) -> np.ndarray:
        """
        Generate finger position data in the format expected by the preprocessor.
        
        Args:
            hand: "left" or "right"
            
        Returns:
            Array of shape (25, 4, 4) with finger positions
        """
        fingertips = np.zeros([25, 4, 4], dtype=np.float32)

        # Get the finger values for this hand [thumb, index, middle]
        finger_values = self._latest_finger_data[hand]

        # Map to the expected format
        # Indices: 0=thumb, 5=index, 10=middle, 15=ring, 20=pinky
        # We only control thumb, index, middle from OpenCV data
        thumb_idx = 4 + 0
        index_idx = 4 + 5
        middle_idx = 4 + 10
        ring_idx = 4 + 15

        # Set positions (1.0 = closed, 0.0 = open)
        fingertips[thumb_idx, 0, 3] = finger_values[0]
        fingertips[index_idx, 0, 3] = finger_values[1]
        fingertips[middle_idx, 0, 3] = finger_values[2]
        fingertips[ring_idx, 0, 3] = 0.0  # Keep ring open

        return fingertips

    def __del__(self):
        """Cleanup on deletion."""
        self.stop_streaming()


if __name__ == "__main__":
    # Test the streamer
    print("Starting OpenCV streamer test...")
    streamer = OpenCVStreamer(udp_port=5005)
    streamer.start_streaming()

    try:
        while True:
            data = streamer.get()
            print("\n" + "="*60)
            print(f"Source: {data.source}")
            print(f"Left wrist position: {data.ik_data['left_wrist'][:3, 3]}")
            print(f"Right wrist position: {data.ik_data['right_wrist'][:3, 3]}")
            print(f"Base height: {data.control_data['base_height_command']}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.stop_streaming()
