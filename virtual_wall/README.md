### virtual_wall ROS2(jazzy) Package

1) launch
ros2 launch virtual_wall virtual_wall.launch.py

2) topic list
/Global_virtual_wall # Type: sensor_msgs/msg/PointCloud2
/Local_virtual_wall  # Type: sensor_msgs/msg/PointCloud2

3) service call
3.1) Create 2 squares

ros2 service call /add_virtual_wall_cmd virtual_wall/srv/AddVirtualWall "{polygons: [
  {points: [{x: 0.5, y: 1.0, z: 0.0}, {x: 1.5, y: 0.5, z: 0.0}, {x: 1.5, y: 1.5, z: 0.0}, {x: 1.0, y: 1.5, z: 0.0}]},
  {points: [{x: 0.0, y: 2.0, z: 0.0}, {x: 1.0, y: 2.0, z: 0.0}, {x: 1.0, y: 2.5, z: 0.0}, {x: 0.0, y: 2.5, z: 0.0}]}
], ids: [1, 2], remove: false}"

3.2) Delete 2nd 1 square from 2 squares

ros2 service call /add_virtual_wall_cmd virtual_wall/srv/AddVirtualWall "{ids: [2], remove: true}"

