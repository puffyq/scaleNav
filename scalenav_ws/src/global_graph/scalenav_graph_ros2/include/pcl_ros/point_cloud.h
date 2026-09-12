#pragma once

// ROS2 has no pcl_ros/point_cloud.h compatibility header. EPIC only uses the
// PCL point-cloud type from this include, so keep the adapter deliberately
// small and middleware-independent.
#include <pcl/point_cloud.h>
