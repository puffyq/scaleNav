from __future__ import annotations

import numpy as np


def accept_local_goal(
    current_goal: np.ndarray | None,
    next_goal: np.ndarray,
    trajectory_valid: bool,
    tolerance: float = 1e-3,
) -> tuple[np.ndarray, bool, bool]:
    """Return goal, changed flag, and uninterrupted trajectory-valid state."""
    if current_goal is not None and np.linalg.norm(current_goal - next_goal) <= tolerance:
        return current_goal, False, trajectory_valid
    return next_goal, True, trajectory_valid


def is_final_subgoal(
    local_goal: np.ndarray | None,
    mission_goal: np.ndarray | None,
    tolerance: float,
) -> bool:
    """Whether EPIC's rolling waypoint has advanced to the mission goal."""
    if local_goal is None or mission_goal is None:
        return False
    return bool(np.linalg.norm(local_goal - mission_goal) <= tolerance)


def mission_goal_for_local_goal(
    local_goal: np.ndarray,
    mission_goal: np.ndarray | None,
    has_separate_mission_goal: bool,
) -> np.ndarray | None:
    """Treat a direct local goal as final when no global planner owns it."""
    if has_separate_mission_goal:
        return mission_goal
    return np.asarray(local_goal).copy()


def mission_arrived(
    position: np.ndarray,
    velocity: np.ndarray,
    mission_goal: np.ndarray | None,
    position_tolerance: float,
    speed_tolerance: float,
) -> bool:
    """Require both spatial arrival and braking before declaring completion."""
    if mission_goal is None:
        return False
    return bool(
        np.linalg.norm(position - mission_goal) <= position_tolerance
        and np.linalg.norm(velocity) <= speed_tolerance
    )


def project_goal_to_fixed_altitude(
    goal: np.ndarray, altitude: float | None
) -> np.ndarray:
    """Apply the same fixed-layer projection used by the EPIC graph."""
    projected = np.asarray(goal, dtype=np.float32).copy()
    if altitude is not None:
        projected[2] = float(altitude)
    return projected
