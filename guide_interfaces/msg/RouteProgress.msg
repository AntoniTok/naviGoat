# Progress of the robot along the current route.
# Published by route_follower on guide/route_progress, once per guide/pose (5 Hz).
# All distances are in metres; positions are measured in the map frame.

# stamp = stamp of the pose this was computed from; frame_id = "map"
std_msgs/Header header

# Index of the route segment the robot is projected onto (0 = first leg).
uint32 segment_index

# Distance along the route from its start to the robot's projected point ("s").
float64 distance_along

# Distance along the route from the projected point to the end.
float64 distance_remaining

# Total length of the route.
float64 total_length

# Signed sideways distance from the route at the projected point.
# Positive = robot is left of the route (looking in the direction of travel),
# negative = right. Counter-clockwise positive, as in REP 103.
float64 cross_track_error

# True once the robot has reached the end of the route. Stays true until a new route is loaded.
bool arrived