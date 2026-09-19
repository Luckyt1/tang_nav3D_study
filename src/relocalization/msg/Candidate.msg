# BTC 候选及粗位姿；尚未经点云精配准确认。
uint64 submap_id
uint64 votes
uint64 inliers
float64 mean_binary_similarity
float64 vertex_rmse
# T_submap_query：把当前查询子图的局部点变换到候选子图。
geometry_msgs/Pose submap_from_query
# T_map_query：把当前查询子图的局部点变换到保存的地图。
geometry_msgs/Pose map_from_query
