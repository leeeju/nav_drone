# VoxelGrid.msg

# Header for timestamp and frame_id
std_msgs/Header header

# The resolution of each voxel in meters
float32 resolution

# The origin of the voxel grid in 3D space
geometry_msgs/Point origin

# Dimensions of the grid
uint32 width    # Number of cells in the X direction
uint32 height   # Number of cells in the Y direction
uint32 depth    # Number of cells in the Z direction

# The data array representing the 3D voxel grid in row-major order
float32[] data
