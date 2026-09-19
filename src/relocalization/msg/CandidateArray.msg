# header.stamp 是查询窗口最后一帧时间；frame_id 是保存地图的参考系名称。
std_msgs/Header header
string odom_frame_id
string query_frame_id
geometry_msgs/Pose odom_from_query
# 本轮新增有效帧数；query_cloud 同时保留附近历史观测。
uint64 accumulated_frames
uint64 query_point_count
uint64 binary_count
uint64 triangle_count
# candidates / no_descriptors / no_match；candidates 不代表最终定位成功。
string status
# 空字符串表示有候选；否则说明失败阶段。
# no_descriptors / insufficient_triangles / insufficient_votes / geometry_rejected
string failure_reason
# 各阶段最强支持数，best_votes 和 best_inliers 不一定来自同一子图。
uint64 best_votes
uint64 best_inliers
uint64 required_votes
uint64 required_inliers
# 只经过三角形边长筛选时，单张子图得到的最多票数。
uint64 best_length_votes
relocalization/Candidate[] candidates
