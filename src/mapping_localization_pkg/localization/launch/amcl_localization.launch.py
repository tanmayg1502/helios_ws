"""AMCL localization in an already-built map: owns map -> odom.

The counterpart to slam_toolbox.launch.py: that one BUILDS a map, this one
REUSES a saved one. The map is read-only -- AMCL never writes to the .pgm.

Runs SEPARATELY from the sensor stack, on top of it:

    ros2 launch sensor_fusion bringup.launch.py
    ros2 launch mapping_localization_pkg amcl_localization.launch.py \
        map:=<path-to>/slam_toolbox_20260728_175429.yaml

DO NOT run this alongside slam_toolbox.launch.py, or rtabmap with
publish_tf_map:=true -- all three publish map -> odom and would fight. The EKF
still owns odom -> base_link in every case.

Starts four nodes. nav2_bringup is not used here; the individual nav2_*
packages are wired up directly:
  map_server        - serves the saved .pgm/.yaml on /map
  amcl              - particle filter, publishes map -> odom + /particlecloud
  lifecycle_manager - drives both through configure -> activate, since they
                      are lifecycle nodes that do not self-activate
  amcl_recovery     - ours. Watches AMCL and drives the rover back to a
                      converged pose on its own whenever it goes lost

THE ROVER RE-LOCALIZES ITSELF. amcl_recovery needs no pose estimate, no
teleop and no service call: it notices that AMCL's covariance has stayed bad,
then spins and drives short legs picked from the live local costmap until the
particle cloud tightens. That covers both startup (no pose yet) and getting
lost mid-run. Thresholds and motion limits are in amcl.yaml.

It REQUIRES Nav2 to be running, because the local costmap it steers by and the
spin / drive_on_heading behaviors it drives through both live there. Without
Nav2 it stays passive and logs why. Set amcl_recovery.enabled false in
amcl.yaml to go back to placing the pose by hand in RViz.

Toggle with: rviz:=true recovery:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the AMCL localization launch description.

    Returns:
        map_server, amcl, the lifecycle_manager that activates them and the
        self-triggering recovery node, plus an optional RViz node.
    """
    pkg = get_package_share_directory("mapping_localization_pkg")
    amcl_yaml = os.path.join(pkg, "localization", "config", "amcl.yaml")
    rviz_config = os.path.join(pkg, "slam_toolbox", "rviz", "slam.rviz")

    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    use_recovery = LaunchConfiguration("recovery")

    # The map path arrives as a launch argument, so it has to override what is
    # in amcl.yaml rather than being baked into it -- a saved map is a run
    # input, not a fixed property of the package.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[amcl_yaml, {"yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_yaml],
    )

    # Both nodes above are lifecycle nodes and neither self-activates.
    # lifecycle_manager transitions them in the listed order (map_server
    # first, so a map is being served before AMCL tries to localize against
    # it) and afterwards watches them via its bond timer.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": autostart,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    # Not a lifecycle node, so it is not in lifecycle_manager's list. It polls
    # /amcl/get_state itself and stays passive until AMCL reports active.
    recovery = Node(
        package="mapping_localization_pkg",
        executable="amcl_recovery_node",
        name="amcl_recovery",
        output="screen",
        parameters=[amcl_yaml],
        condition=IfCondition(use_recovery),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="REQUIRED. Path to the saved map .yaml (the file next "
                "to the .pgm), e.g. "
                "src/mapping_localization_pkg/slam_toolbox/maps/"
                "slam_toolbox_20260728_175429.yaml",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz with the SLAM layout (Map + LaserScan + TF "
                "are what you want to watch here).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Drive map_server and amcl to active automatically.",
            ),
            DeclareLaunchArgument(
                "recovery",
                default_value="true",
                description="Start amcl_recovery, which re-localizes the rover "
                "by itself when AMCL goes lost. Needs Nav2 running.",
            ),
            map_server,
            amcl,
            lifecycle_manager,
            recovery,
            rviz,
        ]
    )
