# Mesh Navigation

The *Mesh Navigation* bundle provides software to perform efficient robot navigation on 3D meshes in ROS.

* Maintainer: [Sebastian Pütz](mailto:spuetz@uos.de)
* Author: [Sebastian Pütz](mailto:spuetz@uos.de)

## Installation from Repository

If you want to install the mesh navigation stack from source please follow the instructions from the 
[pluto_robot](https://github.com/uos/pluto_robot) repository, since we use the robot Pluto as exemplary
robot to perform mesh navigation in simulation but also in the real world.

Otherwise use `sudo apt install ros-melodic-mesh-navigation`

## Pluto in Simulation

You can use Pluto in an outdoor simulation environment. We provides several datasets and the corresponding environments
for the Gazebo simulation. For navigation purposes the corresponding navigation launch file should be started, too. 
The following simulation environments are currently available:

- Botanical Garden at Osnabrück University: `roslaunch pluto_gazebo pluto_botanical_garden.launch`
- Stone Quarry in the Forest in Brockum: `roslaunch pluto_gazebo pluto_stone_quarry.launch`
- Physics building at Osnabrück University: `roslaunch pluto_gazebo pluto_physics.launch`

## Mesh Navigation Stack

This [mesh_navigation](https://github.com/uos/mesh_navigation) stack provides a navigation server for 
[Move Base Flex (MBF)](https://github.com/magazino/move_base_flex). It provides a couple of configuration files and launch 
files to start the navigation server with the configured layer plugins for the layered mesh map, and the configured
planners and controller to perform path planning and motion control in 3D (or more specifically on 2D-manifold). 

The package structure is as follows:

- `mesh_navigation` The corresponding ROS meta package.

- `mbf_mesh_core` contains the plugin interfaces derived from the abstract MBF plugin interfaces to initialize 
  planner and controller plugins with one `mesh_map` instance. It provides the following three interfaces:
  
  - MeshPlanner - `mbf_mesh_core/mesh_planner.h`
  - MeshController - `mbf_mesh_core/mesh_controller.h`
  - MeshRecovery - `mbf_mesh_core/mesh_recovery.h`

- `mbf_mesh_nav` contains the mesh navigation server which build on top of the abstract MBF navigation server.
  It uses the plugin interfaces in `mbf_mesh_core` to load and initialize plugins of the types described above.

- `mesh_map` contains an implementation of a mesh map representation building on top of the mesh data structures
  in [LVR2](https://github.com/uos/lvr2). This package provides a layered mesh map implementation. Layers can be 
  loaded as plugins to allow a highly configurable 3D navigation stack for robots traversing on the ground in outdoor
  and rough terrain.

- `mesh_layers` The package provides a couple of mesh layers to compute the trafficability of the terrain. 
  Furthermore, these plugins have access to the HDF5 map file and can load and store layer information. 
  The mesh layers can be configured for the robots abilities and needs. Currently we provide the following layer plugins:
  
  - HeightDiffLayer - `mesh_layers/height_diff_layer.h`
  - RoughnessLayer - `mesh_layers/roughness_layer.h`
  - InflationLayer - `mesh_layers/inflation_layer.h`
  - SteepnessLayer - `mesh_layers/steepness_layer.h`

- `dijkstra_mesh_planner` contains a mesh planner plugin providing a path planning method based on Dijkstra's algorithm.
  It plans by using the edges of the mesh map. The propagation start a the goal pose, thus a path from every accessed 
  vertex to the goal pose can be computed. This leads in a sub-optimal potential field, which highly depends on the mesh 
  structure.

- `wave_front_planner` contains a Fast Marching Method (FMM) wave front path planner to take the 2D-manifold into account.
  This planner is able to plan over the surface, due to that it results in shorter paths than the `dijkstra_mesh_planner`,
  since it is not restricted to the edges or topology of the mesh. A comparison is shown below.

- `mesh_client` Is an experimental package to load navigation meshes only from a mesh server.

## Mesh Navigation Demo

### Botanical Garden at Osnabrück University

#### Demo Video Botanical Garden

[![Mesh Navigation with Pluto](http://img.youtube.com/vi/qAUWTiqdBM4/0.jpg)](http://www.youtube.com/watch?v=qAUWTiqdBM4)

### Stone Quarry

#### Colored Point Cloud

![StoneQuarryPointCLoud](docs/images/stone_quarry/cloud.png?raw=true "Stone Quarry Point Cloud")

#### Height Diff Layer

![StoneQuarryHeightDiff](docs/images/stone_quarry/height_diff.jpg?raw=true "Stone Quarry Height Diff")

#### Mesh RGB Vertex Colors

![StoneQuarryVertexColors](docs/images/stone_quarry/mesh_rgb.jpg?raw=true "Stone Quarry Vertex Colors")

## Run Mesh Navigation in Simulation

If you want to test the mesh navigation stack with Pluto please use the simulation setup and the corresponding launch
files below for the respective outdoor or rough terrain environment. The mesh tools have to be installed.
We developed the [Mesh Tools](https://github.com/uos/mesh_tools) as a package consisting of message definitions, RViz plugins and tools, as well as a
persistence layer to store such maps. These tools make the benefits of annotated triangle maps available in ROS and
allow to publish, edit and inspect such maps within the existing ROS software stack.

### RVIZ

Run RViz with the preconfigured display panels for mesh navigation. 

```
roscd pluto_navigation
rviz -d rviz/mesh_navigation.rviz
```

### Outdoor Environments

- Botanical Garden at Osnabrück University: 
  
  ```
  rosocre
  roslaunch pluto_gazebo pluto_botanical_garden.launch
  roslaunch pluto_navigation pluto_botanical_garden.launch
  ```

- Stone Quarry in the Forest in Brockum: 
  
  ```
  roscore
  roslaunch pluto_gazebo pluto_stone_quarry.launch
  roslaunch pluto_navigation pluto_stone_quarry.launch
  ```

- Physics Building at Osnabrück University: 
  
  ```
  roscore
  roslaunch pluto_gazebo pluto_physics.launch
  roslaunch pluto_navigation pluto_physics.launch
  roslaunch pluto_navigation navigation_goals.launch goal:=physics1
  ```

### Path Planning and Motion Control

Use the `MeshGoal` tool to select a goal pose on the shown mesh in RViz. 

The planners are compared to each other.

#### Wave Front Planner

![WaveFrontPlanner](docs/images/stone_quarry/fmm_pot.jpg?raw=true "Wave Front Planner")

#### Dijkstra Mesh Planner

![DijkstraMeshPlanner](docs/images/stone_quarry/dijkstra_pot.jpg?raw=true "Dijkstra Mesh Planner")

#### 2D Planner on 2.5 Digital Elevation Map (DEM)

![2D-DEM-Planner](docs/images/stone_quarry/dem_side.jpg?raw=true "2D DEM Planner")

### Available Costlayers

| name            | description                                                                                                                                                                                             | example                                                                                     |
| --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| **height_diff** | The height_diff layer computes the *difference in height* at each vertex.<br/><br/>This layer is useful to avoid *angled areas* like shown in the example.                                              | ![HeightDiffLayer](docs/images/costlayers/height_diff.jpg?raw=true "Height Diff Costlayer") |
| **roughness**   | The roughness layer computes the local *roughness* of the surface surrounding each vertex.<br/><br/>As shown in the example, this layer is useful to *differentiate between paths and overgrown areas*. | ![HeightDiffLayer](docs/images/costlayers/roughness.jpg?raw=true "Height Diff Costlayer")   |
| **steepness**   | The steepness layer calculates the costs based on the angle of the vertex normals.<br/><br/>The example shows a use case of this costlayer by penalizing *steep areas* like stairs or walls             | ![HeightDiffLayer](docs/images/costlayers/steepness.jpg?raw=true "Height Diff Costlayer")   |
| **ridge**       | The ridge costlayer penalizes *ridges on the surface*<br/><br/>It is useful for *agricultural environments* like the field shown in the example.                                                        | ![HeightDiffLayer](docs/images/costlayers/ridge.jpg?raw=true "Height Diff Costlayer")       |
| **inflation**   | The inflation layer *inflates obstacles* detected by other layers.<br/><br/>This is useful to *avoid near proximity* to obstacles and avoid collisions.                                                 | ![HeightDiffLayer](docs/images/costlayers/inflation.jpg?raw=true "Height Diff Costlayer")   |
