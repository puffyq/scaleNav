# Representative direction in the scan/body frame
geometry_msgs/Vector3 dir

# Yaw and elevation bias for limited gaps
float32 yaw_bias
float32 elev_bias

# Spherical angles (rad) and mean range (m)
float32 center_yaw
float32 center_elev
float32 range_mean

# Angular spans (rad) within the subregion
float32 yaw_span
float32 elev_span

# Pixel count in the subregion
int32 size

# 0: OPEN, 1: LIMITED, 2: FREE
uint8 type
