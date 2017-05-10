neo-ros
---
### Neo ROS Driver and Node
This node is publishing a /scan directly.
### Installation
In your ros workspace:
```
git clone https://github.com/micvision/neo-ros.git
```
Then:
```
catkin_make
```
### Usage
1. Run without visible(without rviz):
```
roslaunch neo_ros neo.launch
```
2. Run with rviz:
```
roslaunch neo_ros view_neo_directly_scan.launch
```

