[![Jazzy CI](https://github.com/naturerobots/mesh_navigation/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/naturerobots/mesh_navigation/actions/workflows/jazzy.yaml)
[![Humble CI](https://github.com/naturerobots/mesh_navigation/actions/workflows/humble.yaml/badge.svg)](https://github.com/naturerobots/mesh_navigation/actions/workflows/humble.yaml)

<div align="center" min-width=519px>
  <img src="docs/images/mesh_navigation_logo.png" alt="Mesh Navigation" height=150 />  
</div>
<h4 align="center">Mobile Robot Navigation in 3D Meshes</h4>
<div align="center">
  <a href="https://github.com/naturerobots/mesh_navigation_tutorials">Tutorials</a>
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

<!-- # Contents
* [Publications](#publications)
* [Installation](#installation)
* [Usage Examples and Demos](#usage-examples-and-demos)
* [Software Stack](#software-stack)
* [Mesh Map](#mesh-map)
* [Planners](#planners)
* [Controllers](#controllers)
* [Simulation](#simulation)
* [Maintain and Contribute](#maintain-and-contribute)
* [Build Status](#build-status) -->


# Installation

### ROS Version
This is the active ROS 2 branch of this repository, which targets `humble`.
If your are looking for the old ROS 1 version, checkout the [noetic branch](https://github.com/naturerobots/mesh_navigation/tree/noetic).
**Warning**: The ROS 1 version of mesh_navigation is not maintained anymore.

### Installation from source
* Prerequisite: A working ROS 2 installation
* Go into a ROS 2 workspace's source directory `cd $YOUR_ROS_WS/src`.
* Clone the repo `git clone git@github.com:naturerobots/mesh_navigation.git`
* Get the tutorial's ROS 2 dependencies
  * Clone source dependencies: Run `vcs import --input mesh_navigation/source_dependencies.yaml` in your ROS 2 workspace source directory.
  * Get packaged dependencies: Run `rosdep install --from-paths . --ignore-src -r -y` from within your ROS 2 workspace source directory.
* Build: Go to workspace root `cd $YOUR_ROS_WS` and run `colcon build --packages-up-to mesh_navigation`.

# Usage Examples and Demos

Recommended entrypoint for new users: Check out the **[mesh_navigation_tutorials](https://github.com/naturerobots/mesh_navigation_tutorials/tree/main)** for a ready-to-use mesh navigation stack. Complete with simulated environment, RViz config, mesh nav config, etc.


## Demos

In the following demo videos we used the developed *continuous vector field planner* (CVP).

| Dataset and Description                  | Demo Video                                                                                                               |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| Botanical Garden of Osnabrück University | [![Mesh Navigation with Pluto](http://img.youtube.com/vi/qAUWTiqdBM4/0.jpg)](http://www.youtube.com/watch?v=qAUWTiqdBM4) |
| Stone Quarry in the Forest Brockum       | [![Mesh Navigation with acorn19](http://img.youtube.com/vi/DFmv3wnIxug/0.jpg)](https://youtu.be/DFmv3wnIxug)             |

### Stone Quarry in the Forest in Brockum

| Colored Point Cloud | Height Diff Layer | RGB Vertex Colors |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------- |
| ![StoneQuarryPointCLoud](docs/images/stone_quarry/cloud.png?raw=true "Stone Quarry Point Cloud") | ![StoneQuarryHeightDiff](docs/images/stone_quarry/height_diff.jpg?raw=true "Stone Quarry Height Diff") | ![StoneQuarryVertexColors](docs/images/stone_quarry/mesh_rgb.jpg?raw=true "Stone Quarry Vertex Colors") |

# Software Stack

This **[mesh_navigation](https://github.com/naturerobots/mesh_navigation)** stack provides a navigation server for **[Move Base Flex (MBF)](https://github.com/naturerobots/move_base_flex)**. It provides a couple of configuration files and launch files to start the navigation server with the configured layer plugins for the layered mesh map, and the configured planners and controller to perform path planning and motion control in 3D (or more specifically on 2D-manifold). 

The package structure is as follows:

- `mesh_navigation` The corresponding ROS meta package.

- `mbf_mesh_core` contains the plugin interfaces derived from the abstract MBF plugin interfaces to initialize planner and controller plugins with one `mesh_map` instance. It provides the following three interfaces:
  
  - MeshPlanner - `mbf_mesh_core/mesh_planner.h`
  - MeshController - `mbf_mesh_core/mesh_controller.h`
  - MeshRecovery - `mbf_mesh_core/mesh_recovery.h`

- `mbf_mesh_nav` contains the mesh navigation server which is built on top of the abstract MBF navigation server. It uses the plugin interfaces in `mbf_mesh_core` to load and initialize plugins of the types described above.

- `mesh_map` contains an implementation of a mesh map representation building on top of the generic mesh interface implemented in **[lvr2](https://github.com/uos/lvr2)**. This package provides a layered mesh map implementation. Layers can be loaded as plugins to allow a highly configurable 3D navigation stack for robots traversing on the ground in outdoor and rough terrain.

- `mesh_layers` The package provides a couple of mesh layers to compute trafficability/traversibility properties of the terrain. Furthermore, these plugins have access to the HDF5 map file and can load and store layer information. The mesh layers can be configured for the robots abilities and needs. Currently we provide the following layer plugins:
  
  - HeightDiffLayer - `mesh_layers/HeightDiffLayer`
  - RoughnessLayer - `mesh_layers/RoughnessLayer`
  - SteepnessLayer - `mesh_layers/SteepnessLayer`
  - RidgeLayer - `mesh_layer/RidgeLayer`
  - ClearanceLayer - `mesh_layers/ClearanceLayer`
  - InflationLayer - `mesh_layers/InflationLayer`
  - BorderLayer - `mesh_layers/BorderLayer`

- `dijkstra_mesh_planner` contains a mesh planner plugin providing a path planning method based on Dijkstra's algorithm. It plans by using the edges of the mesh map. The propagation start a the goal pose, thus a path from every accessed vertex to the goal pose can be computed. This leads to a sub-optimal potential field, which highly depends on the mesh structure.

- `cvp_mesh_planner` contains a Fast Marching Method (FMM) wave front path planner to take the 2D-manifold into account. This planner is able to plan over the surface, due to that it results in shorter paths than the `dijkstra_mesh_planner`, since it is not restricted to the edges or topology of the mesh. A comparison is shown below. Please refer to the paper `Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots`.

## Mesh Map

### Mesh Layers

The following table gives an overview of all currently implemented layer plugins available in the stack and the corresponding types to specify for usage in the mesh map configuration. An example mesh map configuration is shown below.

#### Overview of all layers

| Layer               | Plugin Type Specifier         | Description of Cost Computation          | Example Image                                                                           |
| ------------------- | ----------------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------- |
| **HeightDiffLayer** | `mesh_layers/HeightDiffLayer` | local radius based height differences    | ![HeightDiffLayer](docs/images/costlayers/height_diff.jpg?raw=true "Height Diff Layer") |
| **RoughnessLayer**  | `mesh_layers/RoughnessLayer`  | local radius based normal fluctuation    | ![RoughnessLayer](docs/images/costlayers/roughness.jpg?raw=true "Roughness Layer")      |
| **SteepnessLayer**  | `mesh_layers/SteepnessLayer`  | arccos of the normal's z coordinate      | ![SteepnessLayer](docs/images/costlayers/steepness.jpg?raw=true "Steepness Layer")      |
| **RidgeLayer**      | `mesh_layer/RidgeLayer`       | local radius based distance along normal | ![RidgeLayer](docs/images/costlayers/ridge.jpg?raw=true "RidgeLayer")                   |
| **ClearanceLayer**  | `mesh_layers/ClearanceLayer`  | comparison of robot height and clearance along each vertex normal | ![ClearanceLayer](docs/images/costlayers/clearance.jpg?raw=true "Clearance Layer") |
| **InflationLayer**  | `mesh_layers/InflationLayer`  | by distance to a lethal vertex           | ![InflationLayer](docs/images/costlayers/inflation.jpg?raw=true "Inflation Layer")      |
| **BorderLayer** | `mesh_layers/BorderLayer` | give vertices close to the border a certain cost | ![BorderLayer](docs/images/costlayers/border.png?raw=true "Border Layer")   |

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

### MMP Planner

```yaml
  mesh_planner:
    type: 'mmp_planner/MMPPlanner'
```

**Note**: The MMP planner is currently only available with ROS 1 / [mmp-planner](https://github.com/naturerobots/mesh_navigation/tree/mmp-planner) branch.

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

# Publications

Please reference the following papers when using the navigation stack in your scientific work.

#### Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots
```bib
@inproceedings{puetz21cvp,
    author = {Pütz, Sebastian and Wiemann, Thomas and Kleine Piening, Malte and Hertzberg, Joachim},
    title = {Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots},
    booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
    year = 2021,
    url = {https://github.com/uos/mesh_navigation},
    note = {Software available at \url{https://github.com/uos/mesh_navigation}}
}
```

#### Move Base Flex: A Highly Flexible Navigation Framework for Mobile Robots
```bib
@inproceedings{puetz18mbf,
    author = {Sebastian Pütz and Jorge Santos Simón and Joachim Hertzberg},
    title = {{Move Base Flex}: A Highly Flexible Navigation Framework for Mobile Robots},
    booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    year = 2018,
    month = {October},
    url = {https://github.com/magazino/move_base_flex},
    note = {Software available at \url{https://github.com/magazino/move_base_flex}}
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
* [Sebastian Pütz](mailto:sebastian.puetz@naturerobots.com) (Nature Robots, DFKI)

Author: [Sebastian Pütz](mailto:spuetz@uos.de)

We are happy to receive improvements to the mesh navigation stack. Just open an issue. PRs are welcome!
