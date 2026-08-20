from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .depth_query import ValidationState
from .sparse_graph import GraphUpdate, SparseDepthGraph


STATE_RGBA = {
    ValidationState.CERTIFIED.value: (0.15, 0.65, 0.37, 1.0),
    ValidationState.UNVALIDATED.value: (0.89, 0.64, 0.10, 1.0),
    ValidationState.INVALID.value: (0.85, 0.29, 0.29, 1.0),
}


@dataclass(frozen=True)
class GraphVisualizationSnapshot:
    edge_segments: dict[str, tuple[tuple[np.ndarray, np.ndarray], ...]]
    node_points: dict[str, tuple[np.ndarray, ...]]
    certified_path: tuple[np.ndarray, ...]
    optimistic_path: tuple[np.ndarray, ...]
    current: np.ndarray
    goal: np.ndarray
    waypoint: np.ndarray | None


def build_graph_visualization(
    graph: SparseDepthGraph,
    update: GraphUpdate,
    goal_world: np.ndarray,
) -> GraphVisualizationSnapshot:
    goal = _vector(goal_world, "goal_world")
    edge_segments: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {
        state.value: [] for state in ValidationState
    }
    node_points: dict[str, list[np.ndarray]] = {
        state.value: [] for state in ValidationState
    }
    for node in graph.nodes.values():
        node_points[node.state.value].append(node.position_world.copy())
    for edge in graph.edges.values():
        edge_segments[edge.state.value].append(
            (
                graph.nodes[edge.source].position_world.copy(),
                graph.nodes[edge.target].position_world.copy(),
            )
        )

    def path_points(path: tuple[int, ...]) -> tuple[np.ndarray, ...]:
        return tuple(graph.nodes[node_id].position_world.copy() for node_id in path)

    waypoint = update.certified_waypoint_world
    if waypoint is None:
        waypoint = update.optimistic_waypoint_world
    return GraphVisualizationSnapshot(
        edge_segments={key: tuple(value) for key, value in edge_segments.items()},
        node_points={key: tuple(value) for key, value in node_points.items()},
        certified_path=path_points(update.certified_path),
        optimistic_path=path_points(update.optimistic_path),
        current=graph.nodes[update.current_node_id].position_world.copy(),
        goal=goal.copy(),
        waypoint=None if waypoint is None else waypoint.copy(),
    )


def enu_to_ned(point_enu: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    """Invert the Colosseum ROS bridge's NED-to-ENU position conversion."""
    x_enu, y_enu, z_enu = _vector(point_enu, "point_enu")
    return np.array([y_enu, x_enu, -z_enu], dtype=np.float64)


def _vector(values: np.ndarray | list[float] | tuple[float, ...], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain three finite values")
    return result
