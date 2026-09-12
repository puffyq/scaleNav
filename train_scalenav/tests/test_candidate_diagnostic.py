import numpy as np
from scipy.spatial import cKDTree

from evaluate_candidates import _candidate_record


def test_candidate_record_contains_diagnostic_metrics():
    path = np.stack(
        [np.linspace(0.0, 4.0, 9), np.zeros(9), np.full(9, 1.6)], axis=1
    ).astype(np.float32)
    trajectory = np.stack(
        [np.linspace(0.0, 2.0, 11), np.zeros(11), np.full(11, 1.6)], axis=1
    ).astype(np.float32)
    obstacles = np.asarray([[0.0, 2.0, 1.6], [4.0, 2.0, 1.6]], dtype=np.float32)
    result = _candidate_record(
        trajectory, 1.25, 3, path, np.full(9, 1.0, dtype=np.float32), cKDTree(obstacles)
    )
    assert result["primitiveIndex"] == 3
    assert result["score"] == 1.25
    assert len(result["path"]) == 11
    assert "meanCenterlineDistanceM" in result
    assert "maximumCorridorViolationM" in result
