[![Jazzy CI](https://github.com/naturerobots/mesh_navigation/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/naturerobots/mesh_navigation/actions/workflows/jazzy.yaml)
[![Humble CI](https://github.com/naturerobots/mesh_navigation/actions/workflows/humble.yaml/badge.svg)](https://github.com/naturerobots/mesh_navigation/actions/workflows/humble.yaml)

<div align="center" min-width=519px>
  <img src="docs/images/mesh_navigation_logo.png" alt="Mesh Navigation" height=150 />  
</div>
<h4 align="center">Mobile Robot Navigation in 3D Meshes</h4>
<div align="center">
  <a href="https://naturerobots.github.io/mesh_navigation_docs/tutorials/">Tutorials</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://naturerobots.github.io/mesh_navigation_docs/">Documentation</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://www.youtube.com/@nature-robots">Videos</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/naturerobots/move_base_flex">Move Base Flex</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/naturerobots/mesh_tools">Mesh Tools</a>

  <br />
</div>

---

The *Mesh Navigation* bundle provides software for efficient robot navigation on 2D manifolds, which are represented in 3D as triangle meshes. It enables safe navigation in various complex outdoor environments by using a modularly extensible
layered mesh map. Layers can be loaded as plugins representing specific geometric or semantic metrics of the terrain. This allows the incorporation of obstacles in these complex outdoor environments into path and motion motion planning.
The layered *Mesh Map* is integrated with *Move Base Flex (MBF)*, which provides a universal ROS action interface for path planning, motion control, and for recovery behaviors. We also provide additional planner and controller plugins that run on the layered mesh map.

<center><img title="Demo Gif" src="docs/images/demo.gif?raw=true" alt="Demo Gif" width="600"></center>

# Installation

### ROS Version
This is the active ROS 2 branch of this repository, which targets `humble`.
If your are looking for the old ROS 1 version, checkout the [noetic branch](https://github.com/naturerobots/mesh_navigation/tree/noetic).
**Warning**: The ROS 1 version of mesh_navigation is not maintained anymore.

### Installation from source

You need a working ROS 2 installation; we target `humble` and `jazzy` at the moment. Go into a ROS 2 workspace's source directory `cd $YOUR_ROS_WS/src`. Then clone the source code 

```bash
git clone git@github.com:naturerobots/mesh_navigation.git`
```

Get MeshNav's ROS 2 dependencies
* Clone source dependencies: Run `vcs import --input mesh_navigation/source_dependencies.yaml` in your ROS 2 workspace source directory.
* Get packaged dependencies: Run `rosdep install --from-paths . --ignore-src -r -y` from within your ROS 2 workspace source directory.

Build: Go to workspace root `cd $YOUR_ROS_WS` and run 

```bash
colcon build --packages-up-to mesh_navigation
```

# Usage Examples and Demos

**Recommended entrypoint for new users:** Start with the **[mesh_navigation_tutorials](https://naturerobots.github.io/mesh_navigation_docs/tutorials/)**, a ready-to-use mesh navigation stack including simulated environments, RViz setup, and configuration files. As part of the tutorials, we also provide a collection of **virtual worlds** that work even without a robot or powerful hardware: [Explore Virtual Worlds](https://naturerobots.github.io/mesh_navigation_docs/tutorials/tutorial_worlds/).

## Demo Videos

In the following demo videos we used the developed *continuous vector field planner* (CVP).

| Botanical Garden of Osnabrück University                  | Stone Quarry in the Forest Brockum                                                                                                               |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| [![Mesh Navigation with Pluto](http://img.youtube.com/vi/qAUWTiqdBM4/0.jpg)](http://www.youtube.com/watch?v=qAUWTiqdBM4) | [![Mesh Navigation with acorn19](http://img.youtube.com/vi/DFmv3wnIxug/0.jpg)](https://youtu.be/DFmv3wnIxug)             |

#### Stone Quarry in the Forest in Brockum

| Colored Point Cloud | Height Diff Layer | RGB Vertex Colors |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------- |
| ![StoneQuarryPointCLoud](docs/images/stone_quarry/cloud.png?raw=true "Stone Quarry Point Cloud") | ![StoneQuarryHeightDiff](docs/images/stone_quarry/height_diff.jpg?raw=true "Stone Quarry Height Diff") | ![StoneQuarryVertexColors](docs/images/stone_quarry/mesh_rgb.jpg?raw=true "Stone Quarry Vertex Colors") |


# Plugins

MeshNav provides a plugin system so that you can write your own plugins for mesh layers and Mesh-based planning & control. This project already provides some implementations you can use out of the box.

## Mesh Layers

<div class="no-header">

|   |   |  |  |
|:---:|:---:|:---:|:---:|
| ![HeightDiffLayer](docs/images/costlayers/height_diff.jpg?raw=true "Height Diff Layer") | ![RoughnessLayer](docs/images/costlayers/roughness.jpg?raw=true "Roughness Layer") | ![SteepnessLayer](docs/images/costlayers/steepness.jpg?raw=true "Steepness Layer") | ![RidgeLayer](docs/images/costlayers/ridge.jpg?raw=true "RidgeLayer") |
|  HeightDiff | Roughness  |  Steepness |  Ridge |
| ![ClearanceLayer](docs/images/costlayers/clearance.jpg?raw=true "Clearance Layer") |  ![InflationLayer](docs/images/costlayers/inflation.jpg?raw=true "Inflation Layer") | ![BorderLayer](docs/images/costlayers/border.png?raw=true "Border Layer") | ![ObstacleLayer](docs/images/costlayers/obstacle.png?raw=true "ObstacleLayer") |
|  Clearance | Inflation  |  Steepness |  Obstacle |

</div>

[>> More Information <<](https://naturerobots.github.io/mesh_navigation_docs/tutorials/mesh_cost_layers/)

## Planners
Currently the following planners are available:

### Dijkstra Mesh Planner

```yaml
  mesh_planner:
    type: 'dijkstra_mesh_planner/DijkstraMeshPlanner'
```

### Continuous Vector Field Planner

```yaml
  mesh_planner:
    type: 'cvp_mesh_planner/CVPMeshPlanner'
```

The planners are compared to each other.

| Vector Field Planner                                                                        | Dijkstra Mesh Planner                                                                              | ROS Global Planner on 2.5D DEM                                                     |
| ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| ![VectorFieldPlanner](docs/images/stone_quarry/fmm_pot.jpg?raw=true "Vector Field Planner") | ![DijkstraMeshPlanner](docs/images/stone_quarry/dijkstra_pot.jpg?raw=true "Dijkstra Mesh Planner") | ![2D-DEM-Planner](docs/images/stone_quarry/dem_side.jpg?raw=true "2D DEM Planner") |

## Controllers

### Mesh Controller

```yaml
  mesh_controller:
    type: 'mesh_controller/MeshController'
```

### MeshMPPI

```yaml
  mesh_controller:
    type: `mesh_mppi/MeshMPPI`
```

# Related Publications

Please reference the following papers when using the navigation stack in your scientific work.

#### 3D Navigation Mesh Generation for Path Planning in Uneven Terrain

```bib
@inproceedings{puetz20163dnav,
  title     = {3D Navigation Mesh Generation for Path Planning in Uneven Terrain},
  author    = {Pütz, Sebastian and Wiemann, Thomas and Sprickerhof, Jochen and Hertzberg, Joachim},
  booktitle = {9th IFAC Symposium on Intelligent Autonomous Vehicles (IAV)},
  series    = {IFAC-PapersOnLine},
  volume    = {49},
  number    = {15},
  pages     = {212--217},
  year      = {2016},
  doi       = {10.1016/j.ifacol.2016.07.734}
}
```

#### Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots
```bib
@inproceedings{puetz21cvp,
  title     = {Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots},
  author    = {Pütz, Sebastian and Wiemann, Thomas and Kleine Piening, Malte and Hertzberg, Joachim},
  booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2021},
  doi       = {10.1109/ICRA48506.2021.9560981}
}
```

#### Navigation Control & Path Planning for Autonomous Mobile Robots

```bib
@phdthesis{puetz2022diss,
  title  = {Navigation Control \& Path Planning for Autonomous Mobile Robots}
  author = {Sebastian Pütz},
  school = {Universität Osnabrück},
  year   = {2022},
  doi    = {10.48693/69}
}
```

# Related Work

## Mesh Tools

We developed the **[Mesh Tools](https://github.com/naturerobots/mesh_tools)** as a package consisting of message definitions, RViz plugins and tools, as well as a
persistence layer to store such maps. These tools make the benefits of annotated triangle maps available in ROS and
allow to publish, edit and inspect such maps within the existing ROS software stack.

## Mesh Localization

For the necessary localization of the robot relative to the mesh, we recommend using RMCL: [https://github.com/uos/rmcl](https://github.com/uos/rmcl). We presented the combination of both software packages at [ROSCon 2023](https://vimeo.com/879000775):

<a href="https://vimeo.com/879000775" target="_blank" ><img src="docs/images/roscon2023_talk.png" alt="MeshNav ROSCon 2023 Video" width="300px"/></a>

# Maintain and Contribute
Maintainers:
* [Matthias Holoch](mailto:matthias.holoch@naturerobots.com) (Nature Robots)
* [Alexander Mock](https://github.com/amock) (Osnabrück University)
* [Justus Braun](https://github.com/justusbraun) (Osnabrück University)

Authors:
* [Sebastian Pütz](mailto:spuetz@uos.de) (Nature Robots)
* [Alexander Mock](https://github.com/amock) (Osnabrück University)
* [Matthias Holoch](mailto:matthias.holoch@naturerobots.com) (Nature Robots)
* [Justus Braun](https://github.com/justusbraun) (Osnabrück University)

We are happy to receive improvements to the mesh navigation stack. Just open an issue. PRs are welcome!
