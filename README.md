# mesh_navigation
ROS packages for robot navigation on meshes

## Prerequisits
+ Ubuntu 18.4
+ ROS melodic

### github repositories
+ mesh navigation

### software
+ openCV
+ Cuda
<https://www.thomas-krenn.com/de/wiki/CUDA_Installation_unter_Ubuntu>
+ gdb
```
sudo apt install gdb
```
+ xterm
```
sudo apt install xterm 
```


## Launch
+ build the workspace
```
catkin_make [workspace]
```
+ start roscore 
```
roscore
```
+ start RVIZ
```
rviz
```
+ launch the navigation files
```
roslaunch pluto_navigation navigation.launch
```
⋅⋅* The gdb window will pop up. When the script comes to a stop type 'r' and click enter


## RVIZ
At first the display has to be configured. 
+ select TexturedMesh and open the drop down selection
+ for Geometry Topic select /move_base_flex/mesh_map/mesh
+ for Display Type select Vertex Costs and open the selection
+ for Color Scale select Rainbow
+ for Vertex Cost Topic select /move_base_flex/mesh_map/vertex_costs
+ for Vertex Cost Type select Combined Costs


Now save the configuration by clicking on File → save Config

Lastly return to the terminal and restart roslaunch pluto_navigation navigation.launch




