#!/usr/bin/env python3
# =============================================================================
#  Hybrid launch
#
#  how it start up:
#    stage 0: gazebo and wait
#    stage 1: start odom and quantum filter
#    stage 2: start ekf
#    stage 3: start navsat
#
#  sensor -> filter -> ekf -> good pose
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_topic_gate(name: str, topics: list, timeout_s: int = 90, extra_wait_s: int = 0) -> ExecuteProcess:
    """
    Bash process that polls `ros2 topic list` every second until all topics appear.
    exit 0  → topics found (gate passed, next stage will start)
    exit 1  → ros2 not in PATH, or timeout exceeded (triggers Shutdown)
    """
    topic_echos = "\n".join([f'  echo "    {t}"' for t in topics])
    topic_checks = " && ".join([
        f'ros2 topic list 2>/dev/null | grep -qF "{t}"'
        for t in topics
    ])
    extra = (
        f'\n  echo "[{name}] extra warmup {extra_wait_s}s..."\n  sleep {extra_wait_s}'
        if extra_wait_s > 0 else ""
    )

    script = f"""
source /opt/ros/humble/setup.bash 2>/dev/null || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[{name}] ERROR: ros2 not in PATH — source ROS2 before launching"
  exit 1
fi

# Start daemon so ros2 topic list uses DDS cache instead of cold discovery.
# Without daemon, each call spins a fresh node that can miss topics for 3-5s.
ros2 daemon start 2>/dev/null || true
sleep 2

echo "============================================================"
echo "[{name}] waiting for {len(topics)} topic(s):"
{topic_echos}
echo "  timeout={timeout_s}s"
echo "============================================================"

TICK=0
DEADLINE=$((SECONDS + {timeout_s}))
while [ $SECONDS -lt $DEADLINE ]; do
  if {topic_checks}; then
    echo "[{name}] ALL TOPICS FOUND after ${{TICK}}s"{extra}
    exit 0
  fi
  if [ $((TICK % 10)) -eq 0 ] && [ $TICK -gt 0 ]; then
    echo "[{name}] still waiting... (${{TICK}}s elapsed)"
  fi
  TICK=$((TICK+1))
  sleep 1
done

echo "============================================================"
echo "[{name}] TIMEOUT after {timeout_s}s — topics still missing:"
{topic_echos}
echo "  Aborting launch."
echo "============================================================"
exit 1
"""
    return ExecuteProcess(cmd=["bash", "-c", script], output="screen")


def make_stage(gate_proc, success_actions, stage_name: str) -> RegisterEventHandler:
    """
    Returns an event handler: when gate_proc exits
      exit 0 → run success_actions
      exit 1 → log error and emit Shutdown (kills entire launch)
    """
    def on_exit(event, _):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"[{stage_name}] gate FAILED (exit {event.returncode}) — killing launch"),
                EmitEvent(event=Shutdown(reason=f"{stage_name} gate timed out or errored")),
            ]
        return [
            LogInfo(msg=f"[checkpoint] {stage_name} passed → starting next stage"),
        ] + (success_actions if isinstance(success_actions, list) else [success_actions])

    return RegisterEventHandler(
        OnProcessExit(target_action=gate_proc, on_exit=on_exit)
    )


def generate_launch_description():
    pkg = get_package_share_directory("husarion_ugv_description")
    ekf_config = os.path.join(pkg, "config", "hybrid_qf_ekf.yaml")

    # =========================================================================
    # Stage 0: Gazebo + robot + sensor bridges
    # =========================================================================
    gz_lynx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "gz_lynx.launch.py")
        ),
        launch_arguments={
            "world":    LaunchConfiguration("world"),
            "spawn_z":  LaunchConfiguration("spawn_z"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    # /scan no exist, use PointCloud2.
    # wheel odom come later.
    gate_sensors = make_topic_gate(
        "gate_sensors",
        ["/imu/data", "/lidar/points", "/diff_drive_controller/odom"],
        timeout_s=120,
    )

    # =========================================================================
    # Stage 1: Odom sources + Quantum Filter
    # =========================================================================
    odom_sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "odom_sources.launch.py")
        ),
        launch_arguments={
            "use_visual_odom": LaunchConfiguration("use_visual_odom"),
            "use_lidar_odom":  LaunchConfiguration("use_lidar_odom"),
            "use_sim_time":    "true",
        }.items(),
    )

    # Quantum filter make signal clean. it fix covariance too so no relay needed.
    quantum_filter = Node(
        package="husarion_ugv_description",
        executable="quantum_filter_node.py",
        name="quantum_filter_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # wait 3s for rtabmap to warm up.
    gate_qf = make_topic_gate(
        "gate_qf",
        ["/imu/data/filtered", "/gps/fix/filtered", "/odometry/lidar"],
        timeout_s=120,
        extra_wait_s=3,
    )

    # =========================================================================
    # Stage 2: Dual EKF
    # =========================================================================
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_fusion",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/global")],
    )

    gate_ekf = make_topic_gate(
        "gate_ekf",
        ["/odometry/local"],
        timeout_s=60,
        extra_wait_s=5,
    )

    # =========================================================================
    # Stage 3: NavSat Transform
    # =========================================================================
    # map to local! global make bad loop.
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": True}],
        remappings=[
            ("imu",               "imu/data/filtered"),
            ("gps/fix",           "gps/fix/filtered"),
            ("odometry/filtered", "odometry/local"),
        ],
    )

    # =========================================================================
    # Event chain (each gate failure kills the launch)
    # =========================================================================
    return LaunchDescription([
        DeclareLaunchArgument("world",
            default_value=os.path.join(pkg, "worlds", "agriculture.world"),
            description="Path to Gazebo world file"),
        DeclareLaunchArgument("spawn_z",
            default_value="0.35",
            description="Robot spawn height"),
        DeclareLaunchArgument("use_rviz",
            default_value="true",
            description="Launch RViz"),
        DeclareLaunchArgument("use_visual_odom",
            default_value="true",
            description="Launch RTAB-Map RGB-D visual odometry"),
        DeclareLaunchArgument("use_lidar_odom",
            default_value="true",
            description="Launch RTAB-Map ICP lidar odometry"),

        # Stage 0 — immediate
        gz_lynx,
        gate_sensors,

        # Stage 1 — fires when gate_sensors exits 0, kills on exit 1
        make_stage(gate_sensors, [odom_sources, quantum_filter, gate_qf],
                   "sensors→QF"),

        # Stage 2 — fires when gate_qf exits 0
        make_stage(gate_qf, [ekf_local, ekf_map, gate_ekf],
                   "QF→EKF"),

        # Stage 3 — fires when gate_ekf exits 0
        make_stage(gate_ekf, [navsat],
                   "EKF→navsat"),
    ])
