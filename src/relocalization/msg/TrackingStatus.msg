# Status is evaluated independently of matching. Header uses the latest available odometry stamp.
std_msgs/Header header
string odom_frame_id
string sensor_frame_id
# tracking means odometry propagation from a recent accepted ICP fix, not a new ICP match.
bool valid
# waiting_for_fix / tracking / fix_expired / odometry_stale /
# odometry_reset / waiting_for_odometry / odometry_unavailable
string status
builtin_interfaces/Time fix_stamp
# Seconds since the accepted fix; inf if no fix is available.
float64 fix_age
