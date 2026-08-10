# A single detected WhyCode/WhyCon marker.
# Field names match jiriUlr/whycode-ros2 node usage exactly.

int32 id                          # decoded WhyCode ID (-1 if none / WhyCon)
int32 size                        # total marker size [px]
float32 u                         # image x of the marker centre [px]
float32 v                         # image y of the marker centre [px]
float32 angle                     # in-image rotation angle
geometry_msgs/Pose position       # 3D pose (camera-centric when coords_method=0)
geometry_msgs/Vector3 rotation    # Euler angles: x=roll, y=pitch, z=yaw
int32 maxx                        # bounding box
int32 maxy
int32 minx
int32 miny
int32 outer_size                  # outer segment size [px]
int32 inner_size                  # inner segment size [px]
