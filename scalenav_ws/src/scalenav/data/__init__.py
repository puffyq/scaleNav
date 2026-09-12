"""Dataset collection and validation helpers."""

__all__ = [
    "AirSimSnapshotCollector",
    "CaptureConfig",
    "export_static_mesh_point_cloud",
    "GeneratedPerson",
    "generated_person_collision_points",
    "load_generated_people",
    "merge_person_collision_point_cloud",
    "PoseSampler",
    "read_ascii_point_cloud_ply",
    "SceneValidationError",
    "SceneWriter",
    "validate_dataset",
    "write_point_cloud_ply",
]


def __getattr__(name):
    if name in __all__:
        from . import snapshot_dataset

        return getattr(snapshot_dataset, name)
    raise AttributeError(name)
