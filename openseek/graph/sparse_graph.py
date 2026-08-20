from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from collections.abc import Sequence

import numpy as np

from .depth_query import DepthSafeVolumeQuery, ValidationState


@dataclass
class GraphNode:
    node_id: int
    position_world: np.ndarray
    state: ValidationState
    clearance_m: float = math.inf
    semantic_score: float = 0.0
    observations: int = 1


@dataclass
class GraphEdge:
    source: int
    target: int
    state: ValidationState
    length_m: float
    clearance_m: float = math.inf


@dataclass(frozen=True)
class GraphConfig:
    candidate_distance_m: float = 5.0
    candidate_yaw_deg: tuple[float, ...] = (-35.0, -17.5, 0.0, 17.5, 35.0)
    candidate_pitch_deg: tuple[float, ...] = (0.0,)
    merge_radius_m: float = 1.0
    path_cost_weight: float = 0.2
    clearance_reward: float = 0.05
    semantic_reward: float = 0.0
    minimum_progress_m: float = 0.5

    def __post_init__(self) -> None:
        if self.candidate_distance_m <= 0.0 or self.merge_radius_m <= 0.0:
            raise ValueError("candidate distance and merge radius must be positive")
        if not self.candidate_yaw_deg or not self.candidate_pitch_deg:
            raise ValueError("candidate angle sets cannot be empty")


@dataclass(frozen=True)
class GraphUpdate:
    current_node_id: int
    goal_node_id: int
    certified_waypoint_world: np.ndarray | None
    optimistic_waypoint_world: np.ndarray | None
    certified_path: tuple[int, ...]
    optimistic_path: tuple[int, ...]
    added_node_ids: tuple[int, ...]
    state_counts: dict[str, int]


class SparseDepthGraph:
    """Persistent sparse topology whose new edges are checked in the current depth frame."""

    def __init__(self, config: GraphConfig | None = None) -> None:
        self.config = config or GraphConfig()
        self.nodes: dict[int, GraphNode] = {}
        self.edges: dict[tuple[int, int], GraphEdge] = {}
        self.current_node_id: int | None = None
        self.goal_node_id: int | None = None
        self._next_node_id = 0

    def update(
        self,
        *,
        position_world: np.ndarray,
        rotation_body_to_world: np.ndarray,
        goal_world: np.ndarray,
        depth_query: DepthSafeVolumeQuery,
        heatmap: np.ndarray | None = None,
        candidate_directions_body: Sequence[np.ndarray] | None = None,
    ) -> GraphUpdate:
        position = self._vector(position_world, "position_world")
        goal = self._vector(goal_world, "goal_world")
        rotation = np.asarray(rotation_body_to_world, dtype=np.float64)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("rotation_body_to_world must be a finite 3x3 matrix")

        current_id, current_added = self._get_or_add_node(
            position, ValidationState.CERTIFIED, math.inf, 0.0
        )
        self.nodes[current_id].state = ValidationState.CERTIFIED
        self.current_node_id = current_id
        added = [current_id] if current_added else []

        # Keep the goal as an explicit terminal node. A direct edge is always
        # present in the topology; only its validation state decides whether
        # certified planning may use it. Unknown space remains optimistic.
        goal_body = rotation.T @ (goal - position)
        direct_goal_validation = depth_query.validate_segment(
            np.zeros(3), goal_body
        )
        goal_node_state = (
            ValidationState.CERTIFIED
            if direct_goal_validation.state == ValidationState.CERTIFIED
            else ValidationState.UNVALIDATED
        )
        goal_id, goal_added = self._get_or_add_node(
            goal,
            goal_node_state,
            direct_goal_validation.clearance_m,
            0.0,
        )
        self.goal_node_id = goal_id
        if goal_added:
            added.append(goal_id)
        self._upsert_edge(
            current_id,
            goal_id,
            direct_goal_validation.state,
            direct_goal_validation.clearance_m,
        )

        certified_goal_edge = (
            direct_goal_validation.state == ValidationState.CERTIFIED
        )

        directions = (
            [self._vector(direction, "candidate_direction_body")
             for direction in candidate_directions_body]
            if candidate_directions_body is not None
            else self._candidate_directions()
        )
        for direction_body in directions:
            norm = float(np.linalg.norm(direction_body))
            if norm <= 1e-6:
                continue
            direction_body = direction_body / norm
            endpoint_body = direction_body * self.config.candidate_distance_m
            validation = depth_query.validate_segment(np.zeros(3), endpoint_body)
            endpoint_world = position + rotation @ endpoint_body
            semantic_score = self._sample_heatmap(heatmap, depth_query, endpoint_body)
            node_id, was_added = self._get_or_add_node(
                endpoint_world,
                validation.state,
                validation.clearance_m,
                semantic_score,
            )
            if was_added:
                added.append(node_id)
            self._upsert_edge(current_id, node_id, validation.state, validation.clearance_m)
            if node_id != goal_id:
                goal_edge_validation = depth_query.validate_optimistic_segment(
                    endpoint_body, goal_body
                )
                self._upsert_edge(
                    node_id,
                    goal_id,
                    goal_edge_validation.state,
                    goal_edge_validation.clearance_m,
                )
                certified_goal_edge = certified_goal_edge or (
                    goal_edge_validation.state == ValidationState.CERTIFIED
                )

        # The terminal node is certified when at least one visible route reaches
        # it. A blocked direct ray must not poison a certified lateral route.
        if certified_goal_edge:
            self.nodes[goal_id].state = ValidationState.CERTIFIED

        # A reported path must terminate at the explicit goal node. If no
        # start-to-goal path is currently known, keep a local frontier waypoint
        # for the next observation but do not label that frontier as a route.
        certified_path = self.shortest_path(
            current_id, goal_id, allow_unvalidated=False
        )
        optimistic_path = self.shortest_path(
            current_id, goal_id, allow_unvalidated=True
        )
        certified_frontier = self._best_frontier_path(
            goal, allow_unvalidated=False
        )
        optimistic_frontier = self._best_frontier_path(
            goal, allow_unvalidated=True
        )
        return GraphUpdate(
            current_node_id=current_id,
            goal_node_id=goal_id,
            certified_waypoint_world=self._first_waypoint(
                certified_path or certified_frontier
            ),
            optimistic_waypoint_world=self._first_waypoint(
                optimistic_path or optimistic_frontier
            ),
            certified_path=tuple(certified_path),
            optimistic_path=tuple(optimistic_path),
            added_node_ids=tuple(added),
            state_counts=self.state_counts(),
        )

    def shortest_path(
        self, source: int, target: int, *, allow_unvalidated: bool = False
    ) -> list[int]:
        if source not in self.nodes or target not in self.nodes:
            raise KeyError("source and target must be existing graph nodes")
        distances = {source: 0.0}
        parents: dict[int, int] = {}
        queue: list[tuple[float, int]] = [(0.0, source)]
        while queue:
            distance, node_id = heapq.heappop(queue)
            if distance != distances.get(node_id):
                continue
            if node_id == target:
                break
            for neighbor, edge in self._neighbors(node_id):
                if edge.state == ValidationState.INVALID:
                    continue
                if not allow_unvalidated and edge.state != ValidationState.CERTIFIED:
                    continue
                node = self.nodes[neighbor]
                if node.state == ValidationState.INVALID:
                    continue
                if not allow_unvalidated and node.state != ValidationState.CERTIFIED:
                    continue
                candidate = distance + edge.length_m
                if candidate < distances.get(neighbor, math.inf):
                    distances[neighbor] = candidate
                    parents[neighbor] = node_id
                    heapq.heappush(queue, (candidate, neighbor))
        if target not in distances:
            return []
        path = [target]
        while path[-1] != source:
            path.append(parents[path[-1]])
        path.reverse()
        return path

    def state_counts(self) -> dict[str, int]:
        return {
            state.value: sum(node.state == state for node in self.nodes.values())
            for state in ValidationState
        }

    def to_dict(self) -> dict:
        return {
            "currentNodeId": self.current_node_id,
            "goalNodeId": self.goal_node_id,
            "nodes": [
                {
                    "id": node.node_id,
                    "positionWorld": node.position_world.tolist(),
                    "state": node.state.value,
                    "clearanceM": self._finite_or_none(node.clearance_m),
                    "semanticScore": float(node.semantic_score),
                    "observations": node.observations,
                }
                for node in self.nodes.values()
            ],
            "edges": [
                {
                    "source": edge.source,
                    "target": edge.target,
                    "state": edge.state.value,
                    "lengthM": edge.length_m,
                    "clearanceM": self._finite_or_none(edge.clearance_m),
                }
                for edge in self.edges.values()
            ],
        }

    def _candidate_directions(self) -> list[np.ndarray]:
        directions = []
        for pitch_deg in self.config.candidate_pitch_deg:
            pitch = math.radians(pitch_deg)
            for yaw_deg in self.config.candidate_yaw_deg:
                yaw = math.radians(yaw_deg)
                directions.append(
                    np.array(
                        [
                            math.cos(pitch) * math.cos(yaw),
                            math.cos(pitch) * math.sin(yaw),
                            math.sin(pitch),
                        ],
                        dtype=np.float64,
                    )
                )
        return directions

    def _get_or_add_node(
        self,
        position: np.ndarray,
        state: ValidationState,
        clearance_m: float,
        semantic_score: float,
    ) -> tuple[int, bool]:
        nearest = None
        nearest_distance = math.inf
        for node_id, node in self.nodes.items():
            distance = float(np.linalg.norm(node.position_world - position))
            if distance < nearest_distance:
                nearest, nearest_distance = node_id, distance
        if nearest is not None and nearest_distance <= self.config.merge_radius_m:
            node = self.nodes[nearest]
            node.observations += 1
            node.semantic_score = max(node.semantic_score, semantic_score)
            node.clearance_m = min(node.clearance_m, clearance_m)
            if state == ValidationState.CERTIFIED:
                node.state = ValidationState.CERTIFIED
            elif node.state != ValidationState.CERTIFIED:
                node.state = state
            return nearest, False
        node_id = self._next_node_id
        self._next_node_id += 1
        self.nodes[node_id] = GraphNode(
            node_id=node_id,
            position_world=position.copy(),
            state=state,
            clearance_m=clearance_m,
            semantic_score=semantic_score,
        )
        return node_id, True

    def _upsert_edge(
        self,
        source: int,
        target: int,
        state: ValidationState,
        clearance_m: float,
    ) -> None:
        if source == target:
            return
        key = tuple(sorted((source, target)))
        length = float(
            np.linalg.norm(self.nodes[source].position_world - self.nodes[target].position_world)
        )
        self.edges[key] = GraphEdge(source, target, state, length, clearance_m)

    def _neighbors(self, node_id: int):
        for edge in self.edges.values():
            if edge.source == node_id:
                yield edge.target, edge
            elif edge.target == node_id:
                yield edge.source, edge

    def _reachable_paths(self, allow_unvalidated: bool) -> dict[int, list[int]]:
        if self.current_node_id is None:
            return {}
        paths = {self.current_node_id: [self.current_node_id]}
        queue = [self.current_node_id]
        while queue:
            source = queue.pop(0)
            for target, edge in self._neighbors(source):
                if target in paths or edge.state == ValidationState.INVALID:
                    continue
                node = self.nodes[target]
                if node.state == ValidationState.INVALID:
                    continue
                if not allow_unvalidated and (
                    edge.state != ValidationState.CERTIFIED
                    or node.state != ValidationState.CERTIFIED
                ):
                    continue
                paths[target] = paths[source] + [target]
                queue.append(target)
        return paths

    def _best_frontier_path(
        self, goal_world: np.ndarray, *, allow_unvalidated: bool
    ) -> list[int]:
        paths = self._reachable_paths(allow_unvalidated)
        if self.current_node_id is None or not paths:
            return []
        current = self.nodes[self.current_node_id]
        initial_distance = float(np.linalg.norm(current.position_world - goal_world))
        best_path = [self.current_node_id]
        best_score = initial_distance
        for node_id, path in paths.items():
            if node_id == self.current_node_id:
                continue
            node = self.nodes[node_id]
            progress = initial_distance - float(np.linalg.norm(node.position_world - goal_world))
            if progress < self.config.minimum_progress_m:
                continue
            path_length = sum(
                self.edges[tuple(sorted((a, b)))].length_m for a, b in zip(path, path[1:])
            )
            clearance = 0.0 if not math.isfinite(node.clearance_m) else node.clearance_m
            score = (
                float(np.linalg.norm(node.position_world - goal_world))
                + self.config.path_cost_weight * path_length
                - self.config.clearance_reward * clearance
                - self.config.semantic_reward * node.semantic_score
            )
            if score < best_score:
                best_score, best_path = score, path
        return best_path

    def _first_waypoint(self, path: list[int]) -> np.ndarray | None:
        if len(path) < 2:
            return None
        return self.nodes[path[1]].position_world.copy()

    @staticmethod
    def _sample_heatmap(
        heatmap: np.ndarray | None,
        depth_query: DepthSafeVolumeQuery,
        endpoint_body: np.ndarray,
    ) -> float:
        if heatmap is None:
            return 0.0
        values = np.asarray(heatmap, dtype=np.float32)
        if values.ndim != 2:
            raise ValueError("heatmap must be two-dimensional")
        projection = depth_query.project(endpoint_body)
        if projection is None:
            return 0.0
        u, v = projection
        row = int(np.clip(round(v * (values.shape[0] - 1) / (depth_query.height - 1)), 0, values.shape[0] - 1))
        column = int(np.clip(round(u * (values.shape[1] - 1) / (depth_query.width - 1)), 0, values.shape[1] - 1))
        value = float(values[row, column])
        return value if math.isfinite(value) else 0.0

    @staticmethod
    def _vector(value: np.ndarray, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError(f"{name} must contain three finite values")
        return vector

    @staticmethod
    def _finite_or_none(value: float) -> float | None:
        return float(value) if math.isfinite(value) else None
