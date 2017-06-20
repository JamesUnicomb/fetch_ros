include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.hit_probability = 0.85
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability = 0.25

TRAJECTORY_BUILDER_2D.laser_min_range = 0.15
TRAJECTORY_BUILDER_2D.laser_max_range = 12.0
TRAJECTORY_BUILDER_2D.laser_missing_echo_ray_length = 6.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
