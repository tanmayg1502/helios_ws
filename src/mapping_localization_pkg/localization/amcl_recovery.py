"""Domain logic for driving a lost robot until AMCL converges.

No ROS imports. Everything here is plain geometry over a planar cost grid, so
it can be read and unit tested without a ROS context.

The grid this operates on is Nav2's LOCAL costmap, which is built in the `odom`
frame from the live laser only. It never consults `/map`, so it stays correct
while AMCL's pose estimate is still wrong. That is what makes it safe to steer
by while the particle cloud is scattered.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final

# nav_msgs/OccupancyGrid encoding: -1 unknown, 0 free, 100 definitely occupied.
UNKNOWN_COST: Final[int] = -1


@dataclass(frozen=True)
class CostmapView:
    """A read-only planar cost grid in row-major order.

    Mirrors the geometry of nav_msgs/OccupancyGrid without depending on it.

    Attributes:
        width: Grid width in cells.
        height: Grid height in cells.
        resolution: Edge length of one cell, in metres.
        origin_x: World x of the lower-left corner of cell (0, 0), in metres.
        origin_y: World y of that same corner, in metres.
        cells: Costs in row-major order, length `width * height`.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    cells: Sequence[int]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"grid must be non-empty, got {self.width}x{self.height}")
        if self.resolution <= 0.0:
            raise ValueError(f"resolution must be positive, got {self.resolution}")
        if len(self.cells) != self.width * self.height:
            raise ValueError(
                f"expected {self.width * self.height} cells, got {len(self.cells)}"
            )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """Convert world coordinates to (column, row).

        Args:
            x: World x in metres, same frame as `origin_x`.
            y: World y in metres.

        Returns:
            The containing cell, or None if the point lies outside the grid.
        """
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def cost_at(self, col: int, row: int) -> int:
        """Return the cost of one cell, or UNKNOWN_COST if out of bounds."""
        if 0 <= col < self.width and 0 <= row < self.height:
            return self.cells[row * self.width + col]
        return UNKNOWN_COST


@dataclass(frozen=True)
class HeadingScore:
    """One candidate direction and how far the robot could travel along it.

    Attributes:
        heading: Direction in the grid's frame, radians.
        clearance: Distance to the first blocking cell, in metres, capped at
            the ray length that was cast.
    """

    heading: float
    clearance: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi].

    atan2(sin, cos) is used rather than fmod arithmetic because it is exact at
    the +-pi boundary and needs no sign correction.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def ray_clearance(
    grid: CostmapView,
    start_x: float,
    start_y: float,
    heading: float,
    max_range: float,
    lethal_threshold: int,
    unknown_is_blocking: bool,
) -> float:
    """March a ray out from a point and report how far it gets.

    Args:
        grid: The cost grid to march through.
        start_x: Ray origin world x, in metres.
        start_y: Ray origin world y, in metres.
        heading: Ray direction in the grid's frame, radians.
        max_range: Distance at which to stop marching, in metres.
        lethal_threshold: Costs at or above this block the ray.
        unknown_is_blocking: Treat UNKNOWN_COST cells as blocking. True is the
            conservative choice: in a rolling local costmap, unknown means the
            laser has not seen that cell this cycle.

    Returns:
        Distance in metres to the first blocking cell, or `max_range` if the
        ray reached full length unobstructed.
    """
    # Half-cell steps. A full-cell step can skip diagonally past a thin
    # obstacle, because a step of `resolution` along a 45 degree heading
    # advances only 0.707 cells in each axis.
    step = grid.resolution * 0.5
    steps = int(max_range / step)
    dx = math.cos(heading) * step
    dy = math.sin(heading) * step

    for i in range(1, steps + 1):
        cell = grid.world_to_cell(start_x + dx * i, start_y + dy * i)
        if cell is None:
            # Left the grid. The rolling window is the limit of what is known,
            # so stop here rather than claiming clearance we cannot see.
            return i * step
        cost = grid.cost_at(*cell)
        if cost == UNKNOWN_COST:
            if unknown_is_blocking:
                return i * step
        elif cost >= lethal_threshold:
            return i * step

    return max_range


def rank_headings(
    grid: CostmapView,
    robot_x: float,
    robot_y: float,
    candidate_count: int,
    max_range: float,
    lethal_threshold: int,
    unknown_is_blocking: bool,
) -> list[HeadingScore]:
    """Cast rays evenly around the robot and measure clearance along each.

    Args:
        grid: The cost grid.
        robot_x: Robot world x in the grid's frame, in metres.
        robot_y: Robot world y, in metres.
        candidate_count: Number of headings sampled evenly over 2*pi.
        max_range: Ray length, in metres.
        lethal_threshold: Costs at or above this block a ray.
        unknown_is_blocking: See `ray_clearance`.

    Returns:
        One HeadingScore per candidate, in increasing heading order.
    """
    if candidate_count <= 0:
        raise ValueError(f"candidate_count must be positive, got {candidate_count}")

    spacing = 2.0 * math.pi / candidate_count
    return [
        HeadingScore(
            heading=wrap_angle(i * spacing),
            clearance=ray_clearance(
                grid,
                robot_x,
                robot_y,
                wrap_angle(i * spacing),
                max_range,
                lethal_threshold,
                unknown_is_blocking,
            ),
        )
        for i in range(candidate_count)
    ]


def choose_heading(
    scores: Sequence[HeadingScore],
    min_clearance: float,
    current_yaw: float,
    previous_heading: float | None,
    turn_weight: float,
    retrace_penalty: float,
    retrace_cone: float,
) -> float | None:
    """Pick the direction to drive next.

    Scoring is `clearance - turn_weight * |turn| - retrace`, which trades three
    things off:

    * Clearance, in metres. More room is better: a longer leg gives the
      particle filter more distinct viewpoints before the next decision.
    * Turn cost. Rotating burns time and adds odometry error, so among equally
      open directions the one needing the least rotation wins.
    * Retrace penalty. Without it the robot ping-pongs along one corridor,
      because reversing always looks open: it just drove through there.

    Args:
        scores: Candidates from `rank_headings`.
        min_clearance: Reject any candidate with less room than this, in metres.
        current_yaw: Robot's current heading in the grid's frame, radians.
        previous_heading: Heading of the previous leg, or None on the first.
        turn_weight: Metres of clearance one radian of rotation is worth.
        retrace_penalty: Score penalty for reversing, in metres.
        retrace_cone: Half-angle around the reverse direction that the penalty
            applies to, in radians.

    Returns:
        The chosen heading in radians, or None if nothing clears
        `min_clearance` (the robot is boxed in and should rotate instead).
    """
    open_enough = [s for s in scores if s.clearance >= min_clearance]
    if not open_enough:
        return None

    reverse = (
        None if previous_heading is None else wrap_angle(previous_heading + math.pi)
    )

    def total(score: HeadingScore) -> float:
        value = score.clearance
        value -= turn_weight * abs(wrap_angle(score.heading - current_yaw))
        if (
            reverse is not None
            and abs(wrap_angle(score.heading - reverse)) < retrace_cone
        ):
            value -= retrace_penalty
        return value

    return max(open_enough, key=total).heading


def is_converged(
    covariance_xx: float,
    covariance_yy: float,
    covariance_yaw: float,
    max_position_std: float,
    max_yaw_std: float,
) -> bool:
    """Decide whether AMCL's particle cloud has tightened enough to trust.

    The covariance diagonal is in squared units, so each term is compared as a
    standard deviation (its square root) against a threshold in native units.

    Args:
        covariance_xx: Variance of x, in m^2.
        covariance_yy: Variance of y, in m^2.
        covariance_yaw: Variance of yaw, in rad^2.
        max_position_std: Largest acceptable x or y standard deviation, metres.
        max_yaw_std: Largest acceptable yaw standard deviation, radians.

    Returns:
        True when all three are within their thresholds.
    """
    # A scattered global cloud reports huge, occasionally negative-zero terms;
    # clamp at 0 so sqrt never raises on floating point noise.
    return (
        math.sqrt(max(covariance_xx, 0.0)) <= max_position_std
        and math.sqrt(max(covariance_yy, 0.0)) <= max_position_std
        and math.sqrt(max(covariance_yaw, 0.0)) <= max_yaw_std
    )


def is_lift_event(
    wheel_speed: float,
    visual_speed: float,
    max_wheel_speed: float,
    min_visual_speed: float,
) -> bool:
    """Decide whether the wheels and the camera disagree about moving at all.

    Carrying the rover produces a signature nothing else does: the wheels are
    still while the camera sees the world sweep past. Mecanum roller slip does
    NOT reach here, because slip is a scale error on a wheel that is turning
    (measured at 0.9686 of truth), not a stationary wheel under real motion.

    Both arguments are speed magnitudes, so this is direction agnostic: a lift
    registers the same whether the rover is carried forwards, sideways or
    backwards.

    Args:
        wheel_speed: Body speed from the wheel encoders, in m/s.
        visual_speed: Body speed from the ZED's visual-inertial odometry, m/s.
        max_wheel_speed: Wheels at or below this count as stationary, in m/s.
        min_visual_speed: The camera must claim at least this much, in m/s.

    Returns:
        True while the two disagree by more than those thresholds allow.
    """
    return wheel_speed <= max_wheel_speed and visual_speed >= min_visual_speed


def is_traction_loss(
    wheel_speed: float,
    visual_speed: float,
    min_wheel_speed: float,
    max_visual_speed: float,
) -> bool:
    """Decide whether the wheels are turning while the body stays put.

    The mirror image of `is_lift_event`, and the one that applies while the
    rover is under command. Lifting a DRIVING rover does not look like a lift:
    the wheels spin freely in the air and report plenty of motion, so the
    wheels-still test cannot fire. What gives it away is the body not moving.

    Also catches a total loss of traction on the ground, which warrants the
    same response: stop, because the odometry being fed to AMCL is fiction.

    Args:
        wheel_speed: Body speed from the wheel encoders, in m/s.
        visual_speed: Body speed from the ZED's visual-inertial odometry, m/s.
        min_wheel_speed: The wheels must claim at least this much, in m/s.
        max_visual_speed: Body speeds at or below this count as not moving, m/s.

    Returns:
        True while the wheels claim motion the camera cannot see.
    """
    return wheel_speed >= min_wheel_speed and visual_speed <= max_visual_speed
