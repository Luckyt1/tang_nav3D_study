# One result per BTC query; header.stamp is the last input cloud stamp.
std_msgs/Header header
string odom_frame_id
string query_frame_id
bool success
bool converged
string failure_reason
uint64 submap_id
uint64 query_points
uint64 target_points
uint64 inliers
# Fraction of downsampled query points within fine.inlier_distance of the submap.
float64 overlap
# Inlier nearest-neighbor RMSE, in meters; inf when no inliers exist.
float64 rmse
float64 translation_correction
# Radians.
float64 rotation_correction
# Valid only when success is true.
geometry_msgs/Pose map_from_query
geometry_msgs/Transform map_from_odom
