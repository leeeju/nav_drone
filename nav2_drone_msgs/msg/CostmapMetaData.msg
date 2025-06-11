# Costmap metadata

# 표준 헤더 (timestamp + frame_id)
std_msgs/Header header

# 셀 하나의 크기 (미터)
float64 resolution

# X, Y 방향의 격자 개수 또는 크기
float64 width
float64 height

# Z 방향의 격자 개수 또는 크기 (새로 추가)
float64 size_z

# Z 방향의 물리적 깊이 (미터) (새로 추가)
float64 depth

# Costmap의 원점(왼쪽 아래 또는 지정한 기준점)
geometry_msgs/Pose origin
