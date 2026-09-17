#!/usr/bin/env python3
"""ROS adapter: notice when AMCL is lost and drive the rover until it is not.

Started by amcl_localization.launch.py alongside AMCL itself, and needs no
trigger. It watches AMCL's reported covariance and, when that stays bad for
long enough, drives the rover in short legs chosen from the live local costmap
until the particle cloud tightens again. No teleop, no RViz click, no service
call.

Escalation, so information is only thrown away as a last resort:

    1. Explore. AMCL's own recovery_alpha_slow/fast injection is what
       re-seeds particles, and it needs motion to act on.
    2. Still lost -> scatter globally, then explore again.
    3. Still lost -> give up, wait out the cooldown, retry.

All geometry lives in `amcl_recovery`; this file handles parameters, pub/sub,
action clients and message conversion only.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from enum import Enum, auto

import rclpy
import tf2_ros

# Sits beside this file in lib/<pkg>/, which Python puts on sys.path as the
# script's own directory. Absolute, not relative: the installed executable runs
# as a standalone script with no parent package for a relative import to
# resolve against.
from amcl_recovery import (
    CostmapView,
    choose_heading,
    is_converged,
    is_lift_event,
    rank_headings,
    wrap_angle,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import DriveOnHeading, Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Empty, Trigger

# PoseWithCovarianceStamped carries a row-major 6x6 over [x, y, z, roll, pitch,
# yaw], so the diagonal terms we need are at 0, 6*1+1 and 6*5+5.
COVARIANCE_XX: int = 0
COVARIANCE_YY: int = 7
COVARIANCE_YAW: int = 35

# Seconds to keep spinning after Ctrl+C so an in-flight cancel reaches the
# behavior server before the process exits.
SHUTDOWN_CANCEL_TIMEOUT: float = 3.0

# action_msgs/GoalStatus.STATUS_SUCCEEDED. Imported as a literal to keep the
# dependency list to message packages this node already needs.
STATUS_SUCCEEDED: int = 4


class State(Enum):
    """Where the node currently is."""

    MONITORING = auto()  # watching AMCL, not driving
    SURVEYING = auto()  # spinning in place to look around
    TURNING = auto()  # rotating onto a chosen heading
    DRIVING = auto()  # translating along that heading


class AmclRecoveryNode(Node):
    """Detects a lost AMCL and drives the rover until it re-converges."""

    def __init__(self) -> None:
        super().__init__("amcl_recovery")

        self.declare_parameter("enabled", True)
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lost_position_std", 0.60)
        self.declare_parameter("lost_yaw_std", 0.40)
        self.declare_parameter("max_position_std", 0.25)
        self.declare_parameter("max_yaw_std", 0.15)
        self.declare_parameter("lost_confirm_time", 5.0)
        self.declare_parameter("pose_timeout", 30.0)
        self.declare_parameter("cooldown", 30.0)
        self.declare_parameter("lift_detection_enabled", True)
        self.declare_parameter("wheel_odom_topic", "/wheel/odometry")
        self.declare_parameter("visual_odom_topic", "/zed/odom_with_cov")
        self.declare_parameter("lift_max_wheel_speed", 0.02)
        self.declare_parameter("lift_min_visual_speed", 0.15)
        self.declare_parameter("lift_confirm_time", 1.0)
        self.declare_parameter("monitor_period", 1.0)
        self.declare_parameter("max_legs", 12)
        self.declare_parameter("survey_yaw", 2.0 * math.pi)
        self.declare_parameter("leg_distance", 1.5)
        self.declare_parameter("leg_speed", 0.2)
        self.declare_parameter("candidate_count", 36)
        self.declare_parameter("ray_range", 3.0)
        self.declare_parameter("safety_margin", 0.45)
        self.declare_parameter("lethal_threshold", 90)
        self.declare_parameter("unknown_is_blocking", True)
        self.declare_parameter("turn_weight", 0.3)
        self.declare_parameter("retrace_penalty", 1.5)
        self.declare_parameter("retrace_cone", 0.6)
        self.declare_parameter("action_timeout", 60.0)

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def integer(name: str) -> int:
            return int(self.get_parameter(name).value)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._lost_position_std = flt("lost_position_std")
        self._lost_yaw_std = flt("lost_yaw_std")
        self._max_position_std = flt("max_position_std")
        self._max_yaw_std = flt("max_yaw_std")
        self._lost_confirm_time = flt("lost_confirm_time")
        self._pose_timeout = flt("pose_timeout")
        self._cooldown = flt("cooldown")
        self._lift_detection = bool(self.get_parameter("lift_detection_enabled").value)
        self._lift_max_wheel_speed = flt("lift_max_wheel_speed")
        self._lift_min_visual_speed = flt("lift_min_visual_speed")
        self._lift_confirm_time = flt("lift_confirm_time")
        self._max_legs = integer("max_legs")
        self._survey_yaw = flt("survey_yaw")
        self._leg_distance = flt("leg_distance")
        self._leg_speed = flt("leg_speed")
        self._candidate_count = integer("candidate_count")
        self._ray_range = flt("ray_range")
        self._safety_margin = flt("safety_margin")
        self._lethal_threshold = integer("lethal_threshold")
        self._unknown_is_blocking = bool(
            self.get_parameter("unknown_is_blocking").value
        )
        self._turn_weight = flt("turn_weight")
        self._retrace_penalty = flt("retrace_penalty")
        self._retrace_cone = flt("retrace_cone")
        self._action_timeout = flt("action_timeout")

        self._state = State.MONITORING
        self._legs = 0
        self._scattered = False
        self._previous_heading: float | None = None
        self._costmap: OccupancyGrid | None = None
        self._amcl_pose: PoseWithCovarianceStamped | None = None
        self._amcl_active = False
        self._amcl_active_since: float | None = None
        self._lost_since: float | None = None
        self._wheel_speed = 0.0
        self._visual_speed = 0.0
        self._lift_since: float | None = None
        self._was_lifted = False
        self._cooldown_until = 0.0
        # The behavior server owns whatever goal is running. A client that just
        # exits does NOT cancel it, so the rover would keep spinning or driving
        # after a Ctrl+C. Hold the handle so shutdown and ~/abort can cancel it.
        self._active_goal: ClientGoalHandle | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._on_costmap,
            1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self._on_amcl_pose,
            1,
        )

        # The same two topics ekf.yaml fuses as odom0 and odom1. Compared here
        # rather than taken from /odometry/filtered, because the EKF blends them
        # into one number and the disagreement is the whole signal.
        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._on_wheel_odom,
            1,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("visual_odom_topic").value),
            self._on_visual_odom,
            1,
        )

        self._reinit = self.create_client(Empty, "/reinitialize_global_localization")
        self._amcl_state = self.create_client(GetState, "/amcl/get_state")
        self._spin = ActionClient(self, Spin, "spin")
        self._drive = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self.create_service(Trigger, "~/relocalize", self._on_relocalize_request)
        self.create_service(Trigger, "~/abort", self._on_abort_request)

        self.create_timer(flt("monitor_period"), self._monitor)

        self.get_logger().info(
            "amcl_recovery watching. Recovers on its own when AMCL goes lost "
            f"(position std > {self._lost_position_std} m or yaw std > "
            f"{self._lost_yaw_std} rad for {self._lost_confirm_time} s). "
            "Needs Nav2 up for the local costmap and the spin/drive behaviors."
            if self._enabled
            else "amcl_recovery is DISABLED by parameter."
        )

    # --- Subscriptions -----------------------------------------------------

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg

    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._wheel_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

    def _on_visual_odom(self, msg: Odometry) -> None:
        self._visual_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        )

    # --- Monitoring --------------------------------------------------------

    def _monitor(self) -> None:
        """Decide once per period whether AMCL needs rescuing."""
        if not self._enabled or self._state is not State.MONITORING:
            return

        self._refresh_amcl_state()
        now = time.monotonic()

        if not self._amcl_active:
            self._lost_since = None
            return
        if now < self._cooldown_until:
            return
        if self._missing_prerequisites():
            return
        if self._check_lift(now):
            return

        if self._amcl_pose is None:
            # AMCL is active but has never published a pose. It only publishes
            # on a filter update, and an update needs motion, so a stationary
            # rover that was never given a pose sits here forever. That is the
            # lost case, not a healthy one.
            if (
                self._amcl_active_since is not None
                and now - self._amcl_active_since > self._pose_timeout
            ):
                self._begin("no pose from AMCL since it went active")
            return

        position_std, yaw_std = self._reported_std()
        if position_std > self._lost_position_std or yaw_std > self._lost_yaw_std:
            if self._lost_since is None:
                self._lost_since = now
            elif now - self._lost_since >= self._lost_confirm_time:
                self._begin(
                    f"lost for {self._lost_confirm_time:.0f} s "
                    f"(position std {position_std:.2f} m, yaw std {yaw_std:.2f} rad)"
                )
        else:
            # Hysteresis: entering recovery uses the loose lost_* thresholds,
            # leaving it uses the tight max_* ones, so a pose hovering near one
            # value cannot oscillate the node in and out of recovery.
            self._lost_since = None

    def _check_lift(self, now: float) -> bool:
        """Detect the rover being carried, and trigger once it is set down.

        This is independent of AMCL. The covariance test above needs AMCL to
        notice it is lost, which needs a filter update, which needs the EKF to
        have reported motion. Carrying the rover with the camera struggling
        produces no such update, so that path can miss a kidnap entirely. The
        wheels disagreeing with the camera is visible either way.

        Args:
            now: Current monotonic time, in seconds.

        Returns:
            True if a recovery was started, so the caller stops checking.
        """
        if not self._lift_detection:
            return False

        lifted = is_lift_event(
            self._wheel_speed,
            self._visual_speed,
            self._lift_max_wheel_speed,
            self._lift_min_visual_speed,
        )

        if lifted:
            if self._lift_since is None:
                self._lift_since = now
            elif now - self._lift_since >= self._lift_confirm_time:
                self._was_lifted = True
            return False

        self._lift_since = None
        # Trigger on TOUCHDOWN, not on the lift itself: recovering while still
        # in the air would spin the wheels in someone's hands.
        if self._was_lifted:
            self._was_lifted = False
            self._begin("carried and set down")
            return True
        return False

    def _refresh_amcl_state(self) -> None:
        """Poll AMCL's lifecycle state without blocking the executor."""
        if not self._amcl_state.service_is_ready():
            self._amcl_active = False
            self._amcl_active_since = None
            return

        def on_state(future: Future) -> None:
            response = future.result()
            active = (
                response is not None
                and response.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE
            )
            if active and not self._amcl_active:
                self._amcl_active_since = time.monotonic()
            elif not active:
                self._amcl_active_since = None
            self._amcl_active = active

        self._amcl_state.call_async(GetState.Request()).add_done_callback(on_state)

    def _reported_std(self) -> tuple[float, float]:
        """Return AMCL's (position, yaw) standard deviations in m and rad."""
        if self._amcl_pose is None:
            return math.inf, math.inf
        covariance = self._amcl_pose.pose.covariance
        position = max(covariance[COVARIANCE_XX], covariance[COVARIANCE_YY])
        return math.sqrt(max(position, 0.0)), math.sqrt(
            max(covariance[COVARIANCE_YAW], 0.0)
        )

    def _missing_prerequisites(self) -> str:
        """Return a human-readable list of what is not up yet, or empty."""
        missing: list[str] = []
        if self._costmap is None:
            missing.append("no local costmap")
        if not self._spin.server_is_ready():
            missing.append("no spin action server")
        if not self._drive.server_is_ready():
            missing.append("no drive_on_heading action server")
        return ", ".join(missing)

    # --- Manual overrides --------------------------------------------------

    def _on_relocalize_request(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._state is not State.MONITORING:
            response.success = False
            response.message = f"already recovering ({self._state.name})"
            return response
        missing = self._missing_prerequisites()
        if missing:
            response.success = False
            response.message = f"not ready: {missing}"
            return response
        self._begin("requested by service call")
        response.success = True
        response.message = "recovery started"
        return response

    def _on_abort_request(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.message = self.abort("aborted by service call")
        response.success = True
        return response

    def abort(self, reason: str) -> str:
        """Stop the sequence and cancel any behavior still executing.

        Args:
            reason: Logged explanation.

        Returns:
            A short description of what was cancelled.
        """
        was_running = self._state is not State.MONITORING
        self._state = State.MONITORING
        self._cooldown_until = time.monotonic() + self._cooldown
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
            self.get_logger().warn(f"{reason}: cancelling the running behavior")
            return "cancelled the running behavior"
        self.get_logger().info(reason)
        return "sequence stopped" if was_running else "nothing was running"

    def has_active_goal(self) -> bool:
        """True while a spin or drive goal is still owned by the server."""
        return self._active_goal is not None

    # --- Recovery sequence -------------------------------------------------

    def _begin(self, reason: str) -> None:
        self._legs = 0
        self._scattered = False
        self._previous_heading = None
        self._lost_since = None
        self._lift_since = None
        self._was_lifted = False
        self.get_logger().warn(f"starting recovery: {reason}")
        self._survey()

    def _survey(self) -> None:
        """Rotate in place so the laser sweeps the full circle.

        AMCL gates its filter updates on translation OR rotation
        (`update_min_a`, 0.20 rad here), so spinning does drive updates. It also
        covers the 90 degrees the 270 degree Hokuyo cannot see at rest.
        """
        self._state = State.SURVEYING
        goal = Spin.Goal()
        goal.target_yaw = self._survey_yaw
        goal.time_allowance = Duration(seconds=self._action_timeout).to_msg()
        self._send(self._spin, goal, self._evaluate)

    def _evaluate(self) -> None:
        """Check convergence, then stop, escalate, or start another leg."""
        if self._amcl_pose is not None and is_converged(
            self._amcl_pose.pose.covariance[COVARIANCE_XX],
            self._amcl_pose.pose.covariance[COVARIANCE_YY],
            self._amcl_pose.pose.covariance[COVARIANCE_YAW],
            self._max_position_std,
            self._max_yaw_std,
        ):
            position_std, yaw_std = self._reported_std()
            self._settle(
                f"recovered after {self._legs} legs "
                f"(position std {position_std:.2f} m, yaw std {yaw_std:.2f} rad)",
                failed=False,
            )
            return

        if self._legs < self._max_legs:
            self._start_leg()
            return

        if not self._scattered:
            # Exploring alone did not do it. Only now is the current estimate
            # worth discarding: a global scatter throws away whatever partial
            # information AMCL still had.
            self._scattered = True
            self._legs = 0
            self._previous_heading = None
            self.get_logger().warn("exploring did not converge; scattering globally")
            self._reinit.call_async(Empty.Request()).add_done_callback(
                lambda _future: self._survey()
            )
            return

        position_std, yaw_std = self._reported_std()
        self._settle(
            f"could not recover (position std {position_std:.2f} m, "
            f"yaw std {yaw_std:.2f} rad); retrying after {self._cooldown:.0f} s",
            failed=True,
        )

    def _start_leg(self) -> None:
        """Choose a direction from the costmap and rotate onto it."""
        grid = self._as_costmap_view()
        pose = self._robot_pose_in_costmap()
        if grid is None or pose is None:
            self._settle("costmap or robot pose unavailable", failed=True)
            return

        robot_x, robot_y, yaw = pose
        scores = rank_headings(
            grid,
            robot_x,
            robot_y,
            self._candidate_count,
            self._ray_range,
            self._lethal_threshold,
            self._unknown_is_blocking,
        )
        heading = choose_heading(
            scores,
            self._leg_distance + self._safety_margin,
            yaw,
            self._previous_heading,
            self._turn_weight,
            self._retrace_penalty,
            self._retrace_cone,
        )

        if heading is None:
            # Boxed in at this range. Spinning is still informative and cannot
            # collide, so survey again rather than giving up.
            self._legs += 1
            self.get_logger().warn("no direction clears the leg distance; surveying")
            self._survey()
            return

        position_std, yaw_std = self._reported_std()
        self.get_logger().info(
            f"leg {self._legs + 1}/{self._max_legs}: "
            f"position std {position_std:.2f} m, yaw std {yaw_std:.2f} rad"
        )
        self._previous_heading = heading
        self._state = State.TURNING
        goal = Spin.Goal()
        goal.target_yaw = wrap_angle(heading - yaw)
        goal.time_allowance = Duration(seconds=self._action_timeout).to_msg()
        self._send(self._spin, goal, self._drive_leg)

    def _drive_leg(self) -> None:
        """Translate forward along the heading just turned onto."""
        self._state = State.DRIVING
        goal = DriveOnHeading.Goal()
        goal.target.x = self._leg_distance
        goal.speed = self._leg_speed
        goal.time_allowance = Duration(seconds=self._action_timeout).to_msg()
        # No continuation: finishing a drive completes the leg, so the result
        # callback advances the counter and re-evaluates convergence.
        self._send(self._drive, goal, None)

    def _settle(self, message: str, failed: bool) -> None:
        """Return to monitoring, holding off re-triggering for the cooldown."""
        self._state = State.MONITORING
        self._lost_since = None
        self._cooldown_until = time.monotonic() + self._cooldown
        if failed:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    # --- Action plumbing ---------------------------------------------------

    def _send(
        self,
        client: ActionClient,
        goal: Spin.Goal | DriveOnHeading.Goal,
        on_success: Callable[[], None] | None,
    ) -> None:
        """Send a goal and chain the next step, without ever blocking.

        Blocking on a future inside a callback deadlocks a single-threaded
        executor: the future can only be completed by the same executor that is
        waiting on it. Everything here is done-callback driven for that reason.

        Args:
            client: Action client to send through.
            goal: The goal message.
            on_success: Zero-argument callable to run when the goal succeeds,
                or None to advance the leg counter and re-evaluate.
        """

        def on_result(future: Future) -> None:
            self._active_goal = None
            result = future.result()
            if result is None or result.status != STATUS_SUCCEEDED:
                self._settle(f"behavior did not succeed: {result}", failed=True)
                return
            if on_success is None:
                self._legs += 1
                self._evaluate()
            else:
                on_success()

        def on_accepted(future: Future) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._settle("behavior server rejected the goal", failed=True)
                return
            self._active_goal = handle
            handle.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal).add_done_callback(on_accepted)

    # --- Conversions -------------------------------------------------------

    def _as_costmap_view(self) -> CostmapView | None:
        """Convert the latest OccupancyGrid into the ROS-free view type."""
        if self._costmap is None:
            return None
        info = self._costmap.info
        return CostmapView(
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            cells=list(self._costmap.data),
        )

    def _robot_pose_in_costmap(self) -> tuple[float, float, float] | None:
        """Look up the robot's pose in the costmap's own frame.

        Returns:
            (x, y, yaw) in metres and radians, or None if the transform is not
            available. Uses a zero timeout: a blocking lookup inside a callback
            would deadlock the executor feeding the TF buffer.
        """
        if self._costmap is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._costmap.header.frame_id,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"transform lookup failed: {exc}")
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # Planar yaw from a quaternion. The general formula reduces to this
        # because a ground robot's x and y components are ~0.
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return translation.x, translation.y, yaw


def main(args: list[str] | None = None) -> None:
    """Spin the recovery node, cancelling any motion on the way out."""
    rclpy.init(args=args)
    node = AmclRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.abort("interrupted")
        # The cancel is asynchronous, so the executor has to keep running long
        # enough to put it on the wire. Exiting immediately would leave the
        # behavior server driving the rover with nobody left to stop it.
        deadline = time.monotonic() + SHUTDOWN_CANCEL_TIMEOUT
        while time.monotonic() < deadline and node.has_active_goal():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
