# Command sent to the stick. Provisional: frame and units still to agree with team.
std_msgs/Header header
float32 direction   # radians, relative to stick forward; positive = left (counter-clockwise)
float32 strength    # 0.0 (no pull) to 1.0 (max pull)
float32 confidence  # 0.0 to 1.0, how sure navigation is
bool stop           # true = user should stop (e.g. at a crossing)