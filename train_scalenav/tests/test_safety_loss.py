import torch

from loss.safety_loss import SafetyLoss


def test_sdf_query_permutates_world_xyz_to_storage_zyx():
    """grid_sample interprets [x,y,z] against an input stored as [D,H,W]."""
    loss = SafetyLoss.__new__(SafetyLoss)
    loss.voxel_size = 0.2
    loss.d0 = 1.0
    loss.r = 1.0
    loss.eval_points = 30

    # value[z, y, x] = 100*z + 10*y + x
    values = torch.zeros((1, 1, 5, 5, 5), dtype=torch.float32)
    for z in range(5):
        for y in range(5):
            for x in range(5):
                values[0, 0, z, y, x] = 100 * z + 10 * y + x
    loss.sdf_maps = [values]

    # Bypass crop selection: this test isolates coordinate ordering.
    loss.get_batch_sdf = lambda pos, map_id: (
        values,
        torch.zeros((1, 3)),
        torch.full((1, 3), 5.0),
    )
    # world [x=1, y=2, z=1] reads storage [z=1,y=2,x=1] = 121.
    position = torch.tensor([[[0.2, 0.4, 0.2]]], dtype=torch.float32)
    _, distance = loss.get_distance_cost(position, torch.tensor([0]))
    assert torch.allclose(distance, torch.tensor([[121.0]]), atol=1.0e-4)
