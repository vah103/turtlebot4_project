#!/usr/bin/env python3
"""Run real-robot SLAM, record useful ROS data, and save the final map.

This helper is intentionally for manual teleoperation. It launches
mapex_lab/launch/slam_robot.launch.py, records a compact rosbag, waits while the
operator drives the robot from another terminal, then saves the occupancy map
and (when available) the Slam Toolbox pose graph.
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


LAB_DIR = Path(__file__).resolve().parents[1]
ROOT_DIR = LAB_DIR.parent
LAUNCH_FILE = LAB_DIR / "launch" / "slam_robot.launch.py"
DEFAULT_OUTPUT_ROOT = LAB_DIR / "experiments" / "manual_slam"

BAG_TOPICS = [
    "/scan",
    "/odom",
    "/tf",
    "/tf_static",
    "/map",
    "/map_metadata",
    "/imu",
    "/joint_states",
    "/cmd_vel",
    "/cmd_vel_stamped",
    "/battery_state",
    "/dock_status",
]


def run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, text=True, **kwargs)


def ros2(*args: str, **kwargs) -> subprocess.CompletedProcess:
    return run(["ros2", *args], **kwargs)


def check_ros_ready() -> None:
    try:
        result = ros2("topic", "list", capture_output=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"Không gọi được ROS 2: {exc}") from exc
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "ros2 topic list thất bại")

    topics = set(result.stdout.splitlines())
    missing = [name for name in ("/scan", "/odom", "/tf") if name not in topics]
    if missing:
        raise RuntimeError(
            "Dell chưa thấy dữ liệu nền tảng của robot: " + ", ".join(missing)
        )

    # A topic name alone is not enough. Require a real scan and odometry sample.
    for topic in ("/scan", "/odom"):
        probe = run(
            ["timeout", "12", "ros2", "topic", "echo", topic, "--once"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if probe.returncode != 0:
            raise RuntimeError(f"Có {topic} nhưng không nhận được message trong 12 giây")


def stream_process_output(proc: subprocess.Popen, log_path: Path) -> None:
    assert proc.stdout is not None
    with log_path.open("w", encoding="utf-8", buffering=1) as log:
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)


def stop_process_group(proc: subprocess.Popen | None, timeout: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=3.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def wait_for_map(timeout_sec: int = 90) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        probe = run(
            ["timeout", "4", "ros2", "topic", "echo", "/map", "--once"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if probe.returncode == 0:
            return True
        time.sleep(1)
    return False


def save_command_output(path: Path, cmd: list[str]) -> None:
    result = run(cmd, capture_output=True)
    path.write_text(
        result.stdout + ("\nSTDERR:\n" + result.stderr if result.stderr else ""),
        encoding="utf-8",
    )


def save_map(run_dir: Path) -> bool:
    map_prefix = run_dir / "map"
    result = ros2(
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(map_prefix),
        capture_output=True,
    )
    (run_dir / "map_save.log").write_text(
        result.stdout + result.stderr,
        encoding="utf-8",
    )
    return result.returncode == 0 and (run_dir / "map.yaml").exists()


def try_serialize_posegraph(run_dir: Path) -> None:
    services = ros2("service", "list", capture_output=True)
    if services.returncode != 0 or "/slam_toolbox/serialize_map" not in services.stdout.splitlines():
        (run_dir / "posegraph_save.log").write_text(
            "Service /slam_toolbox/serialize_map không có; bỏ qua pose graph.\n",
            encoding="utf-8",
        )
        return

    result = ros2(
        "service",
        "call",
        "/slam_toolbox/serialize_map",
        "slam_toolbox/srv/SerializePoseGraph",
        f"{{filename: '{run_dir / 'posegraph'}'}}",
        capture_output=True,
    )
    (run_dir / "posegraph_save.log").write_text(
        result.stdout + result.stderr,
        encoding="utf-8",
    )


def write_metadata(run_dir: Path, run_id: str, phase: str) -> None:
    git = run(
        ["git", "-C", str(ROOT_DIR), "rev-parse", "HEAD"],
        capture_output=True,
    )
    now = dt.datetime.now().astimezone().isoformat()
    lines = [
        f"phase={phase}",
        f"time={now}",
        f"run_id={run_id}",
        f"git_commit={git.stdout.strip() if git.returncode == 0 else 'unknown'}",
        f"ROS_DISTRO={os.environ.get('ROS_DISTRO', '')}",
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '')}",
        f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '')}",
        f"launch={LAUNCH_FILE}",
        "algorithm=slam.launch.py SLAM settings; real robot; manual teleop; no simulation; no Nav2",
        "bag_topics=" + " ".join(BAG_TOPICS),
    ]
    path = run_dir / "metadata.txt"
    mode = "a" if path.exists() else "w"
    with path.open(mode, encoding="utf-8") as out:
        if mode == "a":
            out.write("\n")
        out.write("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_id",
        nargs="?",
        default="real_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"),
        help="Tên phiên quét, ví dụ room1_run1",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    args = parser.parse_args()

    if not LAUNCH_FILE.exists():
        print(f"ERROR: Không tìm thấy {LAUNCH_FILE}", file=sys.stderr)
        return 2

    run_dir = args.output_root / args.run_id
    if run_dir.exists():
        print(f"ERROR: Run đã tồn tại: {run_dir}", file=sys.stderr)
        return 2
    run_dir.mkdir(parents=True)

    print("=== Real TurtleBot 4 manual SLAM ===")
    print(f"Run: {args.run_id}")
    print(f"Output: {run_dir}")
    print(
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} | "
        f"RMW={os.environ.get('RMW_IMPLEMENTATION', '(unset)')}"
    )

    try:
        check_ros_ready()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3

    write_metadata(run_dir, args.run_id, "start")
    save_command_output(run_dir / "topics_start.txt", ["ros2", "topic", "list", "-t"])
    save_command_output(run_dir / "nodes_start.txt", ["ros2", "node", "list"])

    bag_log = (run_dir / "rosbag.log").open("w", encoding="utf-8")
    bag_proc: subprocess.Popen | None = None
    slam_proc: subprocess.Popen | None = None
    slam_thread: threading.Thread | None = None
    map_ready = False

    try:
        print("Đang bắt đầu rosbag...")
        bag_proc = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(run_dir / "bag"), *BAG_TOPICS],
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

        print("Đang bật SLAM cho robot thật...")
        slam_proc = subprocess.Popen(
            ["ros2", "launch", str(LAUNCH_FILE)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        slam_thread = threading.Thread(
            target=stream_process_output,
            args=(slam_proc, run_dir / "slam.log"),
            daemon=True,
        )
        slam_thread.start()

        print("Đang chờ /map...")
        map_ready = wait_for_map()
        if not map_ready:
            raise RuntimeError("Không nhận được /map sau 90 giây")

        print("\n/map đã sẵn sàng.")
        print("Mở TERMINAL KHÁC để teleop robot.")
        print("Khi quét xong, quay lại terminal này và nhấn ENTER để lưu map + kết thúc record.")
        input("\nNhấn ENTER khi đã quét xong... ")

    except KeyboardInterrupt:
        print("\nNhận Ctrl+C: sẽ thử lưu kết quả hiện tại trước khi dừng.")
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
    finally:
        if map_ready:
            print("Đang lưu occupancy map...")
            if save_map(run_dir):
                print(f"Đã lưu: {run_dir / 'map.yaml'} và map.pgm")
            else:
                print("WARNING: map_saver_cli không lưu được map; xem map_save.log")

            print("Đang lưu pose graph nếu Slam Toolbox hỗ trợ...")
            try_serialize_posegraph(run_dir)

        save_command_output(run_dir / "topics_end.txt", ["ros2", "topic", "list", "-t"])
        save_command_output(run_dir / "nodes_end.txt", ["ros2", "node", "list"])

        print("Đang dừng rosbag...")
        stop_process_group(bag_proc)
        bag_log.close()

        print("Đang dừng SLAM...")
        stop_process_group(slam_proc)
        if slam_thread is not None:
            slam_thread.join(timeout=2)

        write_metadata(run_dir, args.run_id, "end")

    print("\n=== Hoàn tất ===")
    print(f"Kết quả: {run_dir}")
    print("- map.yaml + map.pgm: occupancy map")
    print("- bag/: rosbag cho scan/odom/tf/map/imu/cmd_vel...")
    print("- slam.log: log thuật toán SLAM")
    print("- metadata.txt: thông tin phiên chạy")
    print("- posegraph*: pose graph nếu service serialize_map khả dụng")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
