# Mesh Navigation
ROS packages for robot navigation on meshes

## Prerequisits
+ Ubuntu 18.4
+ ROS melodic

### Software
+ openCV
+ Cuda

## Usage
### Launch
```
roslaunch pluto_navigation navigation.launch
```

### RVIZ
At first the display has to be configured. 
```
roscd pluto_navigation
rviz -d rviz/mesh_navigation.rviz
```

