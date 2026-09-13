"""Small CPU/GPU compatible fallback for RayFronts' scatter dependency.

RayFronts only uses ``scatter(..., dim=0)`` in its geometry helpers. The
upstream torch-scatter wheel is unavailable on this host's CUDA toolchain, so
this fallback keeps the optional RayFronts backend runnable without changing
the original planner dependencies.
"""

import torch


def scatter(src, index, out=None, reduce="sum", dim=0):
    if dim != 0:
        raise NotImplementedError("fallback scatter only supports dim=0")
    if index.ndim != 1 or src.shape[0] != index.shape[0]:
        raise ValueError("index must be one-dimensional and match src")
    if out is None:
        size = int(index.max().item()) + 1 if index.numel() else 0
        out = torch.zeros((size,) + tuple(src.shape[1:]), device=src.device,
                          dtype=src.dtype)
    if reduce in ("sum", "add"):
        out.index_add_(0, index, src)
    elif reduce == "mean":
        out.index_add_(0, index, src)
        counts = torch.zeros(out.shape[0], device=src.device, dtype=src.dtype)
        counts.index_add_(0, index, torch.ones_like(index, dtype=src.dtype))
        out.div_(counts.clamp_min(1).reshape((-1,) + (1,) * (out.ndim - 1)))
    else:
        raise NotImplementedError(f"fallback reduction {reduce!r} is unsupported")
    return out

