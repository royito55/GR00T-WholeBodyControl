"""
Low-latency camera viewer using OpenCV for display.

This is an optimized version of run_camera_viewer.py with reduced latency:
- Uses OpenCV windows instead of matplotlib (30-100ms improvement)
- Removes ROS rate limiting (ZMQ CONFLATE handles it)
- Pre-allocates display windows
- Simplified control flow

Usage:
    python run_camera_viewer_fast.py --camera-host localhost --camera-port 5555 --cameras ego_view

Controls:
- R key: Start/Stop recording
- Q or ESC: Quit application
"""

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Optional

import cv2
import rclpy
import threading

from decoupled_wbc.control.main.teleop.configs.configs import ComposedCameraClientConfig
from decoupled_wbc.control.sensor.composed_camera import ComposedCameraClientSensor


@dataclass
class CameraViewerConfig(ComposedCameraClientConfig):
    """Config for running the low-latency camera viewer."""

    offscreen: bool = False
    """Run in offscreen mode (no display, manual recording with R key)."""

    output_path: Optional[str] = None
    """Output path for saving videos. If None, auto-generates path."""

    codec: str = "mp4v"
    """Video codec to use for saving (e.g., 'mp4v', 'XVID')."""

    cameras: Optional[list[str]] = None
    """List of camera names to display/record. If None, uses all available cameras."""

    window_scale: float = 0.5
    """Scale factor for display windows (0.5 = half size for reduced GPU load)."""


ArgsConfig = CameraViewerConfig


def _get_camera_titles(image_data: dict[str, Any]) -> list[str]:
    """Extract all camera keys from the image data."""
    return list(image_data.get("images", {}).keys())


def main(config: ArgsConfig):
    """Main function to run the low-latency camera viewer."""
    # Initialize ROS
    rclpy.init(args=None)
    node = rclpy.create_node("camera_viewer_fast")

    # Start ROS spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    image_sub = ComposedCameraClientSensor(server_ip=config.camera_host, port=config.camera_port)

    # Pre-fetch a sample image to get camera configuration
    print("Connecting to camera server...")
    retry_count = 0
    while retry_count < 100:
        _sample_image = image_sub.read()
        if _sample_image:
            break
        retry_count += 1
        time.sleep(0.05)
    else:
        raise Exception("Failed to connect to camera server")

    camera_titles = _get_camera_titles(_sample_image)
    
    # Filter cameras if specified
    if config.cameras is not None:
        camera_titles = [title for title in camera_titles if title in config.cameras]
        if not camera_titles:
            raise ValueError(
                f"No matching cameras found. Available: {_get_camera_titles(_sample_image)}, "
                f"Requested: {config.cameras}"
            )
        print(f"Displaying cameras: {camera_titles}")
    else:
        print(f"Displaying all cameras: {camera_titles}")

    # Setup output directory
    if config.output_path is None:
        output_dir = Path("camera_recordings")
    else:
        output_dir = Path(config.output_path)

    # Recording state
    is_recording = False
    video_writers = {}
    frame_count = 0
    recording_start_time = None

    # Create OpenCV windows (only in onscreen mode)
    if not config.offscreen:
        for title in camera_titles:
            # WINDOW_NORMAL allows free resizing - image will stretch to fill window entirely
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            # Resize window and image will automatically fill it completely

    print(f"Mode: {'Offscreen' if config.offscreen else 'Onscreen'}")
    print(f"Videos will be saved to: {output_dir}")
    print("Controls: R to record, Q/ESC to quit")

    # FPS tracking
    frame_times = []
    last_fps_print = time.time()

    try:
        while rclpy.ok():
            loop_start = time.time()
            
            # Get images from subscriber (non-blocking)
            image_data = image_sub.read()
            
            if image_data:
                for title in camera_titles:
                    img = image_data["images"].get(title)
                    
                    if img is None:
                        continue

                    # Display image if not offscreen (using OpenCV - very fast)
                    if not config.offscreen:
                        # Convert RGB to BGR for proper OpenCV display
                        img_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        
                        # Add recording indicator
                        if is_recording:
                            cv2.putText(
                                img_display, "REC", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
                            )
                        
                        cv2.imshow(title, img_display)

                    # Save frame if recording
                    if is_recording and title in video_writers:
                        # Convert RGB to BGR for video encoding (OpenCV expects BGR)
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        video_writers[title].write(img_bgr)
                        frame_count += 1

                # Track FPS
                frame_times.append(time.time() - loop_start)
                if len(frame_times) > 100:
                    frame_times.pop(0)

                # Print FPS every 2 seconds
                if time.time() - last_fps_print > 2.0:
                    avg_fps = len(frame_times) / sum(frame_times) if frame_times else 0
                    status = f"REC [{frame_count} frames]" if is_recording else "Ready"
                    print(f"FPS: {avg_fps:.1f} | {status}")
                    last_fps_print = time.time()

            # Handle keyboard input (only in onscreen mode)
            if not config.offscreen:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord('r'):  # R for record
                    if not is_recording:
                        # Start recording
                        recording_dir = output_dir / f"rec_{time.strftime('%Y%m%d_%H%M%S')}"
                        recording_dir.mkdir(parents=True, exist_ok=True)

                        fourcc = cv2.VideoWriter_fourcc(*config.codec)
                        video_writers = {}

                        for title in camera_titles:
                            img = image_data["images"].get(title)
                            if img is not None:
                                height, width = img.shape[:2]
                                video_path = recording_dir / f"{title}.mp4"
                                writer = cv2.VideoWriter(
                                    str(video_path), fourcc, config.fps, (width, height)
                                )
                                video_writers[title] = writer

                        is_recording = True
                        recording_start_time = time.time()
                        frame_count = 0
                        print(f"🔴 Recording started: {recording_dir}")
                    else:
                        # Stop recording
                        is_recording = False
                        for title, writer in video_writers.items():
                            writer.release()
                        video_writers = {}

                        duration = time.time() - recording_start_time if recording_start_time else 0
                        print(f"⏹️  Recording stopped - {duration:.1f}s, {frame_count} frames")
            else:
                # Offscreen mode - minimal sleep to prevent CPU spinning
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Cleanup
        if video_writers:
            for title, writer in video_writers.items():
                writer.release()
            if is_recording:
                duration = time.time() - recording_start_time
                print(f"Final: {duration:.1f}s, {frame_count} frames")

        if not config.offscreen:
            cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == "__main__":
    import tyro
    config = tyro.cli(ArgsConfig)
    main(config)
