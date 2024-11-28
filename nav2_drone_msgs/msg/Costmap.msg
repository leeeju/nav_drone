# This represents a 3D grid map, in which each cell has an associated cost

std_msgs/Header header

# MetaData for the map
CostmapMetaData metadata

# The cost data, in row-major order, starting with (0,0,0).
float32[] data
