# One person, as people_node.py puts them on the map.
int32 id                        # tracker id; negative for people added by hand with RViz's "Publish Point"
geometry_msgs/Point position    # metres, in the frame of the People message's header
geometry_msgs/Vector3 velocity  # m/s, same frame. Zero for someone standing
