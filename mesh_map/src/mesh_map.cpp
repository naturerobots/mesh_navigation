/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */
#include <XmlRpcException.h>
#include <algorithm>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <functional>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <mesh_client/mesh_client.h>
#include <cmath>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/io/hdf5/MeshIO.hpp>
#include <mesh_map/mesh_map.h>
#include <mesh_map/util.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs_conversions/conversions.h>
#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <organized_fast_mesh.h>
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <lvr2/geometry/Matrix4.hpp>

namespace mesh_map {
    using HDF5MeshIO = lvr2::Hdf5IO<lvr2::hdf5features::ArrayIO, lvr2::hdf5features::ChannelIO,
            lvr2::hdf5features::VariantChannelIO, lvr2::hdf5features::MeshIO>;

    MeshMap::MeshMap(tf2_ros::Buffer &tf_listener, bool subscribe)
            : tf_buffer(tf_buffer), private_nh("~/mesh_map/"), first_config(true), map_loaded(false),
              layer_loader("mesh_map", "mesh_map::AbstractLayer"), mesh_ptr(new lvr2::HalfEdgeMesh<Vector>()) {
        private_nh.param<std::string>("server_url", srv_url, "");
        private_nh.param<std::string>("server_username", srv_username, "");
        private_nh.param<std::string>("server_password", srv_password, "");
        private_nh.param<std::string>("mesh_layer", mesh_layer, "mesh");
        private_nh.param<float>("min_roughness", min_roughness, 0);
        private_nh.param<float>("max_roughness", max_roughness, 0);
        private_nh.param<float>("min_height_diff", min_height_diff, 0);
        private_nh.param<float>("max_height_diff", max_height_diff, 0);
        private_nh.param<float>("bb_min_x", bb_min_x, 0);
        private_nh.param<float>("bb_min_y", bb_min_y, 0);
        private_nh.param<float>("bb_min_z", bb_min_z, 0);
        private_nh.param<float>("bb_max_x", bb_max_x, 0);
        private_nh.param<float>("bb_max_y", bb_max_y, 0);
        private_nh.param<float>("bb_max_z", bb_max_z, 0);

        private_nh.param<std::string>("mesh_file", mesh_file, "");
        private_nh.param<std::string>("mesh_part", mesh_part, "");
        private_nh.param<std::string>("global_frame", global_frame, "map");


        ROS_INFO_STREAM("mesh file is set to: " << mesh_file);

        marker_pub = private_nh.advertise<visualization_msgs::Marker>("marker", 100, true);
        mesh_geometry_pub = private_nh.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1, true);
        vertex_costs_pub = private_nh.advertise<mesh_msgs::MeshVertexCostsStamped>("vertex_costs", 1, false);
        vertex_colors_pub = private_nh.advertise<mesh_msgs::MeshVertexColorsStamped>("vertex_colors", 1, true);
        vector_field_pub = private_nh.advertise<visualization_msgs::Marker>("vector_field", 1, true);
        reconfigure_server_ptr = boost::shared_ptr < dynamic_reconfigure::Server < mesh_map::MeshMapConfig >> (
                new dynamic_reconfigure::Server<mesh_map::MeshMapConfig>(private_nh));
        config_callback = boost::bind(&MeshMap::reconfigureCallback, this, _1, _2);
        reconfigure_server_ptr->setCallback(config_callback);
        this->subscribe = subscribe;
        if (this->subscribe) {
            cloud_sub_ = private_nh.subscribe(config.subscribe_node, 100, &MeshMap::createOFM, this);
            speed_pub = private_nh.advertise<std_msgs::Float64>("speed", 1, false);
            penalty = config.penalty;
            setParamsForSpeedCalc();
            this->global_frame = "base_footprint";

        }
    }

    void MeshMap::setParamsForSpeedCalc(){
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_(buffer_);

        geometry_msgs::TransformStamped base_footprint_to_os_sensor;
        geometry_msgs::TransformStamped os_sensor_to_base_footprintos_sensor;
        base_footprint_to_os_sensor = buffer_.lookupTransform("os_sensor", "base_footprint", ros::Time(0),
                                                              ros::Duration(1.0));
        os_sensor_to_base_footprintos_sensor = buffer_.lookupTransform("base_footprint", "os_sensor", ros::Time(0),
                                                                       ros::Duration(1.0));
        //#TODO outsource in function
        transform = lvr2::Quaternion < lvr2::BaseVector < float
                >> (base_footprint_to_os_sensor.transform.rotation.x, base_footprint_to_os_sensor.transform.rotation.y, base_footprint_to_os_sensor.transform.rotation.z, base_footprint_to_os_sensor.transform.rotation.w);
        transform_to_base = lvr2::Quaternion < lvr2::BaseVector < float
                >> (os_sensor_to_base_footprintos_sensor.transform.rotation.x, os_sensor_to_base_footprintos_sensor.transform.rotation.y, os_sensor_to_base_footprintos_sensor.transform.rotation.z, os_sensor_to_base_footprintos_sensor.transform.rotation.w);

        left_wheel = lvr2::BaseVector<float>(config.min_x, -(config.left_wheel), 0);
        right_wheel = lvr2::BaseVector<float>(config.min_x, config.right_wheel, 0);
        width_of_intresst = lvr2::BaseVector<float>(0, 0.3, 0) ;
        depth_of_intresst = lvr2::BaseVector<float>(0, 0, -30);
        hight_of_intresst = lvr2::BaseVector<float>(0, 0, config.max_z) ;
        length_of_intresst = lvr2::BaseVector<float>(20, 0, 0);

        area_of_interesst_left[0] = left_wheel + width_of_intresst + depth_of_intresst;
        area_of_interesst_left[1] = left_wheel - width_of_intresst + depth_of_intresst;

        area_of_interesst_left[2] = left_wheel + width_of_intresst + hight_of_intresst;
        area_of_interesst_left[3] = left_wheel - width_of_intresst + hight_of_intresst;

        area_of_interesst_left[4] = left_wheel + width_of_intresst + depth_of_intresst + length_of_intresst;
        area_of_interesst_left[5] = left_wheel - width_of_intresst + depth_of_intresst + length_of_intresst;

        area_of_interesst_left[6] = left_wheel + width_of_intresst + hight_of_intresst + length_of_intresst;
        area_of_interesst_left[7] = left_wheel - width_of_intresst + hight_of_intresst + length_of_intresst;

        area_of_interesst_right[0] = right_wheel + width_of_intresst + depth_of_intresst;
        area_of_interesst_right[1] = right_wheel - width_of_intresst + depth_of_intresst;

        area_of_interesst_right[2] = right_wheel + width_of_intresst + hight_of_intresst;
        area_of_interesst_right[3] = right_wheel - width_of_intresst + hight_of_intresst;

        area_of_interesst_right[4] = right_wheel + width_of_intresst + depth_of_intresst + length_of_intresst;
        area_of_interesst_right[5] = right_wheel - width_of_intresst + depth_of_intresst + length_of_intresst;

        area_of_interesst_right[6] = right_wheel + width_of_intresst + hight_of_intresst + length_of_intresst;
        area_of_interesst_right[7] = right_wheel - width_of_intresst + hight_of_intresst + length_of_intresst;
        lvr2::Matrix4 <lvr2::BaseVector<float>> to_os_sensor = transform.getMatrix();
        to_os_sensor[3] = base_footprint_to_os_sensor.transform.translation.x;
        to_os_sensor[7] = base_footprint_to_os_sensor.transform.translation.y;
        to_os_sensor[11] = base_footprint_to_os_sensor.transform.translation.z;

        roboter_polyeder[0] = lvr2::BaseVector<float>(config.min_x, config.roboter_wheelbase, config.roboter_ground_clearance);
        roboter_polyeder[1] = lvr2::BaseVector<float>(config.min_x, -config.roboter_wheelbase, config.roboter_ground_clearance);
        roboter_polyeder[2] = lvr2::BaseVector<float>(config.min_x, -config.roboter_wheelbase, config.roboter_hight);
        roboter_polyeder[3] = lvr2::BaseVector<float>(config.min_x, config.roboter_wheelbase, config.roboter_hight);
        roboter_polyeder[4] = lvr2::BaseVector<float>(20, config.roboter_wheelbase, config.roboter_ground_clearance);
        roboter_polyeder[5] = lvr2::BaseVector<float>(20, -config.roboter_wheelbase, config.roboter_ground_clearance);
        roboter_polyeder[6] = lvr2::BaseVector<float>(20, -config.roboter_wheelbase, config.roboter_hight);
        roboter_polyeder[7] = lvr2::BaseVector<float>(20, config.roboter_wheelbase, config.roboter_hight);

        matrixTransform = transform_to_base.getMatrix();
        matrixTransform[3] = os_sensor_to_base_footprintos_sensor.transform.translation.x;
        matrixTransform[7] = os_sensor_to_base_footprintos_sensor.transform.translation.y;
        matrixTransform[11] = os_sensor_to_base_footprintos_sensor.transform.translation.z;
    }

    void MeshMap::createOFM(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        int step = 2;
        divider = 0;
        if (i % 5 == 0) {
            result = 0;
            std::ofstream out;
            out.open("/home/lukas/newtest/speedpub/time.txt", std::ios::app);

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            lvr2::PointBuffer pointBuffer;
            mesh_msgs_conversions::fromPointCloud2ToPointBuffer(*cloud, pointBuffer);
            OrganizedFastMeshGenerator ofmg = OrganizedFastMeshGenerator(pointBuffer, cloud->height, cloud->width, step,
                                                                         step, area_of_interesst_left,
                                                                         area_of_interesst_right, matrixTransform);
            checkleathleObjectsbetweenWheels(pointBuffer);
            ofmg.setEdgeThreshold(config.edgeThreshold);
            lvr2::MeshBufferPtr mesh_buffer_ptr(new lvr2::MeshBuffer);
            mesh_msgs::MeshVertexColorsStamped color_msg;
            ofmg.getMesh(*mesh_buffer_ptr, color_msg);

            mesh_msgs::MeshGeometry mesh_map;
            mesh_msgs_conversions::fromMeshBufferToMeshGeometryMessage(mesh_buffer_ptr, mesh_map);
            *mesh_ptr = lvr2::HalfEdgeMesh < lvr2::BaseVector < float >> (mesh_buffer_ptr);
            this->readMap();
            this->vertex_colors_pub.publish(color_msg);
            publishSpeedoverAllVertex();

            out << cloud->header.stamp << std::endl;
            out.close();

        }
        i++;
    }


    void MeshMap::checkleathleObjectsbetweenWheels(lvr2::PointBuffer &cloudBuffer) {


        float soft = config.softcap;
        lvr2::floatArr cloudPoints = cloudBuffer.getPointArray();
        float threshold = config.threshouldSpeed;
        for (int i = 0; i < cloudBuffer.numPoints() * 3; i += 3) {
            if (cloudPoints[i] != 0 || cloudPoints[i + 1] != 0 || cloudPoints[i + 2] != 0) {
                lvr2::BaseVector<float> p(
                        cloudPoints[i] * matrixTransform[0] + cloudPoints[i + 1] * matrixTransform[1] +
                        cloudPoints[i + 2] * matrixTransform[2] + matrixTransform[3],
                        cloudPoints[i] * matrixTransform[4] + cloudPoints[i + 1] * matrixTransform[5] +
                        cloudPoints[i + 2] * matrixTransform[6] + matrixTransform[7],
                        cloudPoints[i] * matrixTransform[8] + cloudPoints[i + 1] * matrixTransform[9] +
                        cloudPoints[i + 2] * matrixTransform[10] + matrixTransform[11]); // point at (x,y)
                if (isInsideBox(p, roboter_polyeder)) {

                    if (p.x < 40) {
                        result = std::numeric_limits<float>::infinity();
                        divider += 1;

                    } else if (p.x < config.threshouldSpeed) {
                        result = (result * p.x / threshold);
                        divider += (penalty * (1 - (p.x / threshold)));
                    }
                }
            }
        }

    }


    bool MeshMap::isInsideBox(lvr2::BaseVector<float> p, lvr2::BaseVector<float> *vertices) {
        // Berechne die minimalen und maximalen Grenzwerte der Koordinaten des Quaders
        lvr2::BaseVector<float> bMin, bMax;
        bMin.x = bMax.x = vertices[0].x;
        bMin.y = bMax.y = vertices[0].y;
        bMin.z = bMax.z = vertices[0].z;
        for (int i = 1; i < 8; i++) {
            if (vertices[i].x < bMin.x) bMin.x = vertices[i].x;
            if (vertices[i].x > bMax.x) bMax.x = vertices[i].x;
            if (vertices[i].y < bMin.y) bMin.y = vertices[i].y;
            if (vertices[i].y > bMax.y) bMax.y = vertices[i].y;
            if (vertices[i].z < bMin.z) bMin.z = vertices[i].z;
            if (vertices[i].z > bMax.z) bMax.z = vertices[i].z;
        }

        // Überprüfe, ob der Punkt innerhalb des Quaders liegt
        if (p.x < bMin.x || p.x > bMax.x ||
            p.y < bMin.y || p.y > bMax.y ||
            p.z < bMin.z || p.z > bMax.z)
            return false;
        else
            return true;
    }


    bool MeshMap::readMap() {
        if (!subscribe) {
            ROS_INFO_STREAM("server url: " << srv_url);
            bool server = false;

            if (!srv_url.empty()) {
                server = true;

                mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(
                        new mesh_client::MeshClient(srv_url, srv_username, srv_password, mesh_layer));
                auto mesh_client_ptr = std::static_pointer_cast<mesh_client::MeshClient>(mesh_io_ptr);

                mesh_client_ptr->setBoundingBox(bb_min_x, bb_min_y, bb_min_z, bb_max_x, bb_max_y, bb_max_z);
                mesh_client_ptr->addFilter("roughness", min_roughness, max_roughness);
                mesh_client_ptr->addFilter("height_diff", min_height_diff, max_height_diff);
            } else if (!mesh_file.empty() && !mesh_part.empty()) {
                ROS_INFO_STREAM("Load \"" << mesh_part << "\" from file \"" << mesh_file << "\"...");
                HDF5MeshIO *hdf_5_mesh_io = new HDF5MeshIO();
                hdf_5_mesh_io->open(mesh_file);
                hdf_5_mesh_io->setMeshName(mesh_part);
                mesh_io_ptr = std::shared_ptr<lvr2::AttributeMeshIOBase>(hdf_5_mesh_io);
            } else {
                ROS_ERROR_STREAM("Could not open file or server connection!");
                return false;
            }

            if (server) {
                ROS_INFO_STREAM("Start reading the mesh from the server '" << srv_url);
            } else {
                ROS_INFO_STREAM(
                        "Start reading the mesh part '" << mesh_part << "' from the map file '" << mesh_file << "'...");
            }

            auto mesh_opt = mesh_io_ptr->getMesh();

            if (mesh_opt) {
                *mesh_ptr = mesh_opt.get();
                ROS_INFO_STREAM(
                        "The mesh has been loaded successfully with " << mesh_ptr->numVertices() << " vertices and "
                                                                      << mesh_ptr->numFaces() << " faces and "
                                                                      << mesh_ptr->numEdges() << " edges.");

                adaptor_ptr = std::make_unique<NanoFlannMeshAdaptor>(*mesh_ptr);
                kd_tree_ptr = std::make_unique<KDTree>(3, *adaptor_ptr, nanoflann::KDTreeSingleIndexAdaptorParams(10));
                kd_tree_ptr->buildIndex();
                ROS_INFO_STREAM("The k-d tree has been build successfully!");
            } else {
                ROS_ERROR_STREAM(
                        "Could not load the mesh '" << mesh_part << "' from the map file '" << mesh_file << "' ");
                return false;
            }

            vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);
            edge_weights = lvr2::DenseEdgeMap<float>(mesh_ptr->nextEdgeIndex(), 0);
            invalid = lvr2::DenseVertexMap<bool>(mesh_ptr->nextVertexIndex(), false);

            // TODO read and write uuid
            boost::uuids::random_generator gen;
            boost::uuids::uuid uuid = gen();
            uuid_str = boost::uuids::to_string(uuid);

            auto face_normals_opt = mesh_io_ptr->getDenseAttributeMap < lvr2::DenseFaceMap < Normal >> ("face_normals");

            if (face_normals_opt) {
                face_normals = face_normals_opt.get();
                ROS_INFO_STREAM("Found " << face_normals.numValues() << " face normals in map file.");
            } else {
                ROS_INFO_STREAM("No face normals found in the given map file, computing them...");
                face_normals = lvr2::calcFaceNormals(*mesh_ptr);
                ROS_INFO_STREAM("Computed " << face_normals.numValues() << " face normals.");
                if (mesh_io_ptr->addDenseAttributeMap(face_normals, "face_normals")) {
                    ROS_INFO_STREAM("Saved face normals to map file.");
                } else {
                    ROS_ERROR_STREAM("Could not save face normals to map file!");
                }
            }

            auto vertex_normals_opt =
                    mesh_io_ptr->getDenseAttributeMap < lvr2::DenseVertexMap < Normal >> ("vertex_normals");

            if (vertex_normals_opt) {
                vertex_normals = vertex_normals_opt.get();
                ROS_INFO_STREAM("Found " << vertex_normals.numValues() << " vertex normals in map file!");
            } else {
                ROS_INFO_STREAM("No vertex normals found in the given map file, computing them...");
                vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
                if (mesh_io_ptr->addDenseAttributeMap(vertex_normals, "vertex_normals")) {
                    ROS_INFO_STREAM("Saved vertex normals to map file.");
                } else {
                    ROS_ERROR_STREAM("Could not save vertex normals to map file!");
                }
            }

            mesh_geometry_pub.publish(
                    mesh_msgs_conversions::toMeshGeometryStamped<float>(*mesh_ptr, global_frame, uuid_str,
                                                                        vertex_normals));

            ROS_INFO_STREAM("Try to read edge distances from map file...");
            auto edge_distances_opt = mesh_io_ptr->getAttributeMap < lvr2::DenseEdgeMap < float >> ("edge_distances");

            if (edge_distances_opt) {
                ROS_INFO_STREAM("Vertex distances have been read successfully.");
                edge_distances = edge_distances_opt.get();
            } else {
                ROS_INFO_STREAM("Computing edge distances...");
                edge_distances = lvr2::calcVertexDistances(*mesh_ptr);
                ROS_INFO_STREAM("Saving " << edge_distances.numValues() << " edge distances to map file...");

                if (mesh_io_ptr->addAttributeMap(edge_distances, "edge_distances")) {
                    ROS_INFO_STREAM("Saved edge distances to map file.");
                } else {
                    ROS_ERROR_STREAM("Could not save edge distances to map file!");
                }
            }

            ROS_INFO_STREAM("Load layer plugins...");
            if (!loadLayerPlugins()) {
                ROS_FATAL_STREAM("Could not load any layer plugin!");
                return false;
            }

            ROS_INFO_STREAM("Initialize layer plugins...");
            if (!initLayerPlugins()) {
                ROS_FATAL_STREAM("Could not initialize plugins!");
                return false;
            }

            sleep(1);

            combineVertexCosts();
            publishCostLayers();
            publishVertexColors();

            map_loaded = true;
            return true;
        }
            //create and compute Layers for a subscribet Mesh
        else {
            ROS_INFO_STREAM("The mesh has been create successfully with " << mesh_ptr->numVertices() << " vertices and "
                                                                          << mesh_ptr->numFaces() << " faces and "
                                                                          << mesh_ptr->numEdges() << " edges.");

            adaptor_ptr = std::make_unique<NanoFlannMeshAdaptor>(*mesh_ptr);
            kd_tree_ptr = std::make_unique<KDTree>(3, *adaptor_ptr, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kd_tree_ptr->buildIndex();
            edge_weights = lvr2::DenseEdgeMap<float>(mesh_ptr->nextEdgeIndex(), 0);
            invalid = lvr2::DenseVertexMap<bool>(mesh_ptr->nextVertexIndex(), false);
            vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);
            // TODO read and write uuid
            boost::uuids::random_generator gen;
            boost::uuids::uuid uuid = gen();
            uuid_str = boost::uuids::to_string(uuid);

            face_normals = lvr2::calcFaceNormals(*mesh_ptr);
            vertex_normals = lvr2::calcVertexNormals(*mesh_ptr, face_normals);
            edge_distances = lvr2::calcVertexDistances(*mesh_ptr);

            //Load (create) and init new Layers if new layer createt for this map
            if (!map_loaded) {
                if (!loadLayerPlugins()) {
                    ROS_FATAL_STREAM("Could not load any layer plugin!");
                    return false;
                }
                if (!initLayerPlugins()) {
                    ROS_FATAL_STREAM("Could not initialize plugins!");
                    return false;
                }
            }
                //if the layers are created befor compute new layercosts
            else {
                lethals.clear();
                lethal_indices.clear();

                for (auto &layer: layers) {
                    auto &layer_plugin = layer.second;
                    const auto &layer_name = layer.first;
                    std::set <lvr2::VertexHandle> empty;
                    layer_plugin->updateLethal(lethals, empty);
                    layer_plugin->computeLayer(!subscribe);
                    lethal_indices[layer_name].insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
                    lethals.insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
                }
            }


            combineVertexCosts();
            mesh_geometry_pub.publish(
                    mesh_msgs_conversions::toMeshGeometryStamped<float>(*mesh_ptr, global_frame, uuid_str,
                                                                        vertex_normals));
            publishCostLayers();
            map_loaded = true;
            return true;
        }
    }

    bool MeshMap::loadLayerPlugins() {
        XmlRpc::XmlRpcValue plugin_param_list;
        if (!private_nh.getParam("layers", plugin_param_list)) {
            ROS_WARN_STREAM("No layer plugins configured! - Use the param \"layers\" "
                            "in the namespace \""
                                    << private_nh.getNamespace()
                                    << "\". \"layers\" must be must be a list of "
                                       "tuples with a name and a type.");
            return false;
        }

        try {
            for (int i = 0; i < plugin_param_list.size(); i++) {
                XmlRpc::XmlRpcValue elem = plugin_param_list[i];

                std::string name = elem["name"];
                std::string type = elem["type"];

                typename AbstractLayer::Ptr plugin_ptr;
                if (layer_names.find(name) != layer_names.end()) {
                    ROS_ERROR_STREAM("The plugin \"" << name << "\" has already been loaded! Names must be unique!");
                    return false;
                }

                try {
                    plugin_ptr = layer_loader.createInstance(type);
                }
                catch (pluginlib::LibraryLoadException &e) {
                    ROS_ERROR_STREAM(e.what());
                }

                if (plugin_ptr) {
                    std::pair<std::string, typename mesh_map::AbstractLayer::Ptr> elem(name, plugin_ptr);

                    layers.push_back(elem);
                    layer_names.insert(elem);

                    ROS_INFO_STREAM("The layer plugin with the type \""
                                            << type << "\" has been loaded successfully under the name \"" << name
                                            << "\".");
                } else {
                    ROS_ERROR_STREAM(
                            "Could not load the layer plugin with the name \"" << name << "\" and the type \"" << type
                                                                               << "\"!");
                }
            }
        }
        catch (XmlRpc::XmlRpcException &e) {
            ROS_ERROR_STREAM("Invalid parameter structure. The \"layers\" parameter "
                             "has to be a list of structs "
                                     << "with fields \"name\" and \"type\"!");
            ROS_ERROR_STREAM(e.getMessage());
            return false;
        }
        // is there any layer plugin loaded for the map?
        return !layers.empty();
    }

    void MeshMap::layerChanged(const std::string &layer_name) {
        std::lock_guard <std::mutex> lock(layer_mtx);

        ROS_INFO_STREAM("Layer \"" << layer_name << "\" changed.");

        lethals.clear();

        ROS_INFO_STREAM("Combine underlining lethal sets...");

        // TODO pre-compute combined lethals upto a layer level
        auto layer_iter = layers.begin();
        for (; layer_iter != layers.end(); layer_iter++) {
            ROS_INFO_STREAM("Layer \"" << layer_name << "\" published.");
            // TODO add lethal and removae lethal sets
            lethals.insert(layer_iter->second->lethals().begin(), layer_iter->second->lethals().end());
            // TODO merge with std::set_merge
            if (layer_iter->first == layer_name)
                break;
        }

        vertex_costs_pub.publish(
                mesh_msgs_conversions::toVertexCostsStamped(layer_iter->second->costs(), mesh_ptr->numVertices(),
                                                            layer_iter->second->defaultValue(), layer_iter->first,
                                                            global_frame, uuid_str));

        if (layer_iter != layers.end())
            layer_iter++;

        ROS_INFO_STREAM("Combine  lethal sets...");

        for (; layer_iter != layers.end(); layer_iter++) {
            // TODO add lethal and remove lethal sets as param
            layer_iter->second->updateLethal(lethals, lethals);

            lethals.insert(layer_iter->second->lethals().begin(), layer_iter->second->lethals().end());

            vertex_costs_pub.publish(
                    mesh_msgs_conversions::toVertexCostsStamped(layer_iter->second->costs(), mesh_ptr->numVertices(),
                                                                layer_iter->second->defaultValue(), layer_iter->first,
                                                                global_frame, uuid_str));
        }

        ROS_INFO_STREAM("Found " << lethals.size() << " lethal vertices");
        ROS_INFO_STREAM("Combine layer costs...");

        combineVertexCosts();
        // TODO new lethals old lethals -> renew potential field! around this areas
    }

    bool MeshMap::initLayerPlugins() {
        lethals.clear();
        lethal_indices.clear();

        std::shared_ptr <mesh_map::MeshMap> map(this);
        for (auto &layer: layers) {
            auto &layer_plugin = layer.second;
            const auto &layer_name = layer.first;

            auto callback = [this](const std::string &layer_name) { layerChanged(layer_name); };

            if (!layer_plugin->initialize(layer_name, callback, map, mesh_ptr, mesh_io_ptr)) {
                ROS_ERROR_STREAM("Could not initialize the layer plugin with the name \"" << layer_name << "\"!");
                return false;
            }

            std::set <lvr2::VertexHandle> empty;
            layer_plugin->updateLethal(lethals, empty);
            if (!subscribe) {
                if (!layer_plugin->readLayer()) {
                    layer_plugin->computeLayer(!subscribe);
                }
            } else {
                layer_plugin->computeLayer(!subscribe);
            }

            lethal_indices[layer_name].insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
            lethals.insert(layer_plugin->lethals().begin(), layer_plugin->lethals().end());
        }
        return true;
    }

    void MeshMap::combineVertexCosts() {
        ROS_INFO_STREAM("Combining costs...");

        float combined_min = std::numeric_limits<float>::max();
        float combined_max = std::numeric_limits<float>::min();

        vertex_costs = lvr2::DenseVertexMap<float>(mesh_ptr->nextVertexIndex(), 0);

        bool hasNaN = false;
        for (auto layer: layers) {
            const auto &costs = layer.second->costs();
            float min, max;
            mesh_map::getMinMax(costs, min, max);
            const float norm = max - min;
            const float factor = private_nh.param<float>(layer.first + "/factor", 1.0);
            const float norm_factor = factor / norm;


            const float default_value = layer.second->defaultValue();
            hasNaN = false;
            for (auto vH: mesh_ptr->vertices()) {
                const float cost = costs.containsKey(vH) ? costs[vH] : default_value;
                if (std::isnan(cost))
                    hasNaN = true;
                vertex_costs[vH] += factor * cost;


                if (std::isfinite(cost)) {
                    combined_max = std::max(combined_max, vertex_costs[vH]);
                    combined_min = std::min(combined_min, vertex_costs[vH]);
                }
            }
            if (hasNaN)
                ROS_ERROR_STREAM("Layer \"" << layer.first << "\" contains NaN values!");
        }

        const float combined_norm = combined_max - combined_min;

        for (auto vH: lethals) {
            vertex_costs[vH] = std::numeric_limits<float>::infinity();
        }

        vertex_costs_pub.publish(
                mesh_msgs_conversions::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str));

        hasNaN = false;

        for (auto eH: mesh_ptr->edges()) {
            // Get both Vertices of the current Edge
            std::array<lvr2::VertexHandle, 2> eH_vHs = mesh_ptr->getVerticesOfEdge(eH);
            const lvr2::VertexHandle &vH1 = eH_vHs[0];
            const lvr2::VertexHandle &vH2 = eH_vHs[1];
            // Get the Riskiness for the current Edge (the maximum value from both
            // Vertices)
            if (config.layer_factor != 0) {
                if (std::isinf(vertex_costs[vH1]) || std::isinf(vertex_costs[vH2])) {
                    edge_weights[eH] = edge_distances[eH];
                    edge_weights[eH] = std::numeric_limits<float>::infinity();
                } else {
                    float cost_diff = std::fabs(vertex_costs[vH1] - vertex_costs[vH2]);

                    float vertex_factor = config.layer_factor * cost_diff;
                    if (std::isnan(vertex_factor))
                        ROS_INFO_STREAM("NaN: v1:" << vertex_costs[vH1] << " v2:" << vertex_costs[vH2]
                                                   << " vertex_factor:" << vertex_factor << " cost_diff:" << cost_diff);
                    edge_weights[eH] = edge_distances[eH] * (1 + vertex_factor);
                }
            } else {
                edge_weights[eH] = edge_distances[eH];
            }
        }


        ROS_INFO("Successfully combined costs!");
    }

    void MeshMap::findLethalByContours(const int &min_contour_size, std::set <lvr2::VertexHandle> &lethals) {
        int size = lethals.size();
        std::vector <std::vector<lvr2::VertexHandle>> contours;
        findContours(contours, min_contour_size);
        for (auto contour: contours) {
            lethals.insert(contour.begin(), contour.end());
        }
        ROS_INFO_STREAM("Found " << lethals.size() - size << " lethal vertices as contour vertices");
    }

    void MeshMap::findContours(std::vector <std::vector<lvr2::VertexHandle>> &contours, int min_contour_size) {
        ROS_INFO_STREAM("Find contours...");

        std::vector <std::vector<lvr2::VertexHandle>> tmp_contours;

        array<lvr2::OptionalFaceHandle, 2> facepair;
        lvr2::SparseEdgeMap<bool> usedEdges(false);
        for (auto eHStart: mesh_ptr->edges()) {
            lvr2::SparseVertexMap<bool> usedVertices(false);
            lvr2::SparseEdgeMap<bool> local_usedEdges(false);
            int count = 0;

            // Look for border Edges
            facepair = mesh_ptr->getFacesOfEdge(eHStart);

            // If border Edge found
            if ((!facepair[0] || !facepair[1]) && !usedEdges[eHStart]) {
                std:
                vector <lvr2::VertexHandle> contour;
                // Set vector which links to the following Edge
                array<lvr2::VertexHandle, 2> vertexPair = mesh_ptr->getVerticesOfEdge(eHStart);
                lvr2::VertexHandle vH = vertexPair[1];
                vector <lvr2::EdgeHandle> curEdges;
                lvr2::EdgeHandle eHTemp = eHStart;
                bool moving = true;
                bool vertex_flag = false;

                // While the conotur did not come full circle
                while (moving) {
                    moving = false;
                    usedEdges.insert(eHTemp, true);
                    local_usedEdges.insert(eHTemp, true);
                    // Set vector which links to the following Edge
                    vertexPair = mesh_ptr->getVerticesOfEdge(eHTemp);
                    // Eliminate the possibility to choose the previous Vertex
                    if (vH != vertexPair[0]) {
                        vH = vertexPair[0];
                    } else if (vH != vertexPair[1]) {
                        vH = vertexPair[1];
                    }

                    // Add the current Vertex to the contour
                    usedVertices.insert(vH, true);
                    count++;
                    contour.push_back(vH);
                    mesh_ptr->getEdgesOfVertex(vH, curEdges);

                    // Look for other edge of vertex that is a border Edge
                    for (auto eHT: curEdges) {
                        if (!usedEdges[eHT] && !local_usedEdges[eHT]) {
                            facepair = mesh_ptr->getFacesOfEdge(eHT);
                            if (!facepair[0] || !facepair[1]) {
                                eHTemp = eHT;
                                moving = true;
                                continue;
                            }
                        }
                    }
                }
                // Add contour to list of contours
                if (contour.size() > min_contour_size) {
                    contours.push_back(contour);
                }
            }
        }

        ROS_INFO_STREAM("Found " << contours.size() << " contours.");
    }

    void MeshMap::setVectorMap(lvr2::DenseVertexMap <mesh_map::Vector> &vector_map) {
        this->vector_map = vector_map;
    }

    boost::optional <Vector> MeshMap::directionAtPosition(
            const lvr2::VertexMap <lvr2::BaseVector<float>> &vector_map,
            const std::array<lvr2::VertexHandle, 3> &vertices,
            const std::array<float, 3> &barycentric_coords) {
        const auto &a = vector_map.get(vertices[0]);
        const auto &b = vector_map.get(vertices[1]);
        const auto &c = vector_map.get(vertices[2]);

        if (a || b || c) {
            lvr2::BaseVector<float> vec;
            if (a) vec += a.get() * barycentric_coords[0];
            if (b) vec += b.get() * barycentric_coords[1];
            if (c) vec += c.get() * barycentric_coords[2];
            if (std::isfinite(vec.x) && std::isfinite(vec.y) && std::isfinite(vec.z))
                return vec;
            else
                ROS_ERROR_THROTTLE(0.3, "vector map contains invalid vectors!");
        } else {
            ROS_ERROR_THROTTLE(0.3, "vector map does not contain any of the corresponding vectors");
        }
        return boost::none;
    }

    float MeshMap::costAtPosition(const std::array<lvr2::VertexHandle, 3> &vertices,
                                  const std::array<float, 3> &barycentric_coords) {
        return costAtPosition(vertex_costs, vertices, barycentric_coords);
    }

    float MeshMap::costAtPosition(const lvr2::DenseVertexMap<float> &costs,
                                  const std::array<lvr2::VertexHandle, 3> &vertices,
                                  const std::array<float, 3> &barycentric_coords) {
        const auto &a = costs.get(vertices[0]);
        const auto &b = costs.get(vertices[1]);
        const auto &c = costs.get(vertices[2]);

        if (a && b && c) {
            std::array<float, 3> costs = {a.get(), b.get(), c.get()};
            return mesh_map::linearCombineBarycentricCoords(costs, barycentric_coords);
        }
        return std::numeric_limits<float>::quiet_NaN();
    }

    void MeshMap::publishDebugPoint(const Vector pos, const std_msgs::ColorRGBA &color, const std::string &name) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = mapFrame();
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Vector3 scale;
        scale.x = 0.05;
        scale.y = 0.05;
        scale.z = 0.05;
        marker.scale = scale;

        geometry_msgs::Pose p;
        p.position.x = pos.x;
        p.position.y = pos.y;
        p.position.z = pos.z;
        marker.pose = p;
        marker.color = color;
        marker_pub.publish(marker);
    }

    void MeshMap::publishDebugFace(const lvr2::FaceHandle &face_handle, const std_msgs::ColorRGBA &color,
                                   const std::string &name) {
        const auto &vertices = mesh_ptr->getVerticesOfFace(face_handle);
        visualization_msgs::Marker marker;
        marker.header.frame_id = mapFrame();
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Vector3 scale;
        scale.x = 1.0;
        scale.y = 1.0;
        scale.z = 1.0;
        marker.scale = scale;

        for (auto vertex: vertices) {
            auto &pos = mesh_ptr->getVertexPosition(vertex);
            geometry_msgs::Point p;
            p.x = pos.x;
            p.y = pos.y;
            p.z = pos.z;
            marker.points.push_back(p);
            marker.colors.push_back(color);
        }
        marker_pub.publish(marker);
    }

    void MeshMap::publishVectorField(const std::string &name,
                                     const lvr2::DenseVertexMap <lvr2::BaseVector<float>> &vector_map,
                                     const bool publish_face_vectors) {
        publishVectorField(name, vector_map, vertex_costs, {}, publish_face_vectors);
    }

    void MeshMap::publishCombinedVectorField() {
        lvr2::DenseVertexMap <Vector> vertex_vectors;
        lvr2::DenseFaceMap <Vector> face_vectors;

        vertex_vectors.reserve(mesh_ptr->nextVertexIndex());
        face_vectors.reserve(mesh_ptr->nextFaceIndex());

        for (auto layer_iter: layer_names) {
            lvr2::DenseFaceMap <uint8_t> vector_field_faces(mesh_ptr->nextFaceIndex(), 0);
            AbstractLayer::Ptr layer = layer_iter.second;
            auto opt_vec_map = layer->vectorMap();
            if (!opt_vec_map)
                continue;

            const auto &vecs = opt_vec_map.get();
            for (auto vH: vecs) {
                auto opt_val = vertex_vectors.get(vH);
                vertex_vectors.insert(vH, opt_val ? opt_val.get() + vecs[vH] : vecs[vH]);
                for (auto fH: mesh_ptr->getFacesOfVertex(vH))
                    vector_field_faces[fH]++;
            }

            for (auto fH: vector_field_faces) {
                if (vector_field_faces[fH] != 3)
                    continue;

                const auto &vertices = mesh_ptr->getVertexPositionsOfFace(fH);
                const auto &vertex_handles = mesh_ptr->getVerticesOfFace(fH);
                mesh_map::Vector center = (vertices[0] + vertices[1] + vertices[2]) / 3;
                std::array<float, 3> barycentric_coords;
                float dist;
                if (mesh_map::projectedBarycentricCoords(center, vertices, barycentric_coords, dist)) {
                    auto opt_val = face_vectors.get(fH);
                    auto vec_at = layer->vectorAt(vertex_handles, barycentric_coords);
                    if (vec_at != Vector()) {
                        face_vectors.insert(fH, opt_val ? opt_val.get() + vec_at : vec_at);
                    }
                }
            }
        }
    }

    void MeshMap::publishVectorField(const std::string &name,
                                     const lvr2::DenseVertexMap <lvr2::BaseVector<float>> &vector_map,
                                     const lvr2::DenseVertexMap<float> &values,
                                     const std::function<float(float)> &cost_function,
                                     const bool publish_face_vectors) {
        const auto &mesh = this->mesh();
        const auto &vertex_costs = vertexCosts();
        const auto &face_normals = faceNormals();

        visualization_msgs::Marker vector_field;

        geometry_msgs::Pose pose;
        pose.position.x = pose.position.y = pose.position.z = 0;
        pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
        pose.orientation.w = 1;
        vector_field.pose = pose;

        vector_field.type = visualization_msgs::Marker::LINE_LIST;
        vector_field.header.frame_id = mapFrame();
        vector_field.header.stamp = ros::Time::now();
        vector_field.ns = name;
        vector_field.scale.x = 0.01;
        vector_field.color.a = 1;
        vector_field.id = 0;

        vector_field.colors.reserve(2 * vector_map.numValues());
        vector_field.points.reserve(2 * vector_map.numValues());

        unsigned int cnt = 0;
        unsigned int faces = 0;

        lvr2::DenseFaceMap <uint8_t> vector_field_faces(mesh.numFaces(), 0);
        std::set <lvr2::FaceHandle> complete_faces;

        for (auto vH: vector_map) {
            const auto &dir_vec = vector_map[vH];
            const float len2 = dir_vec.length2();
            if (len2 == 0 || !std::isfinite(len2)) {
                ROS_DEBUG_STREAM_THROTTLE(0.3, "Found invalid direction vector in vector field \"" << name
                                                                                                   << "\". Ignoring it!");
                continue;
            }

            auto u = mesh.getVertexPosition(vH);
            auto v = u + dir_vec * 0.1;

            u.z = u.z + 0.01;
            v.z = v.z + 0.01;

            if (!std::isfinite(u.x) || !std::isfinite(u.y) || !std::isfinite(u.z) || !std::isfinite(v.x) ||
                !std::isfinite(v.y) || !std::isfinite(v.z)) {
                continue;
            }
            vector_field.points.push_back(toPoint(u));
            vector_field.points.push_back(toPoint(v));

            const float value = cost_function ? cost_function(values[vH]) : values[vH];
            std_msgs::ColorRGBA color = getRainbowColor(value);
            vector_field.colors.push_back(color);
            vector_field.colors.push_back(color);

            cnt++;
            // vector.header.seq = cnt;
            // vector.id = cnt++;
            // vector_field.markers.push_back(vector);
            try {
                for (auto fH: mesh.getFacesOfVertex(vH)) {
                    if (++vector_field_faces[fH] == 3) {
                        faces++;
                        complete_faces.insert(fH);
                    }
                }
            }
            catch (lvr2::PanicException exception) {
                invalid.insert(vH, true);
            }
        }

        size_t invalid_cnt = 0;
        for (auto vH: invalid) {
            if (invalid[vH])
                invalid_cnt++;
        }

        if (invalid_cnt > 0) {
            ROS_WARN_STREAM("Found " << invalid_cnt << " non manifold vertices!");
        }
        ROS_INFO_STREAM("Found " << faces << " complete vector faces!");

        if (publish_face_vectors) {
            vector_field.points.reserve(faces + cnt);

            for (auto fH: complete_faces) {
                const auto &vertices = mesh.getVertexPositionsOfFace(fH);
                const auto &vertex_handles = mesh.getVerticesOfFace(fH);
                mesh_map::Vector center = (vertices[0] + vertices[1] + vertices[2]) / 3;
                std::array<float, 3> barycentric_coords;
                float dist;
                if (mesh_map::projectedBarycentricCoords(center, vertices, barycentric_coords, dist)) {
                    boost::optional <mesh_map::Vector> dir_opt = directionAtPosition(vector_map, vertex_handles,
                                                                                     barycentric_coords);
                    if (dir_opt) {
                        const float &cost = costAtPosition(values, vertex_handles, barycentric_coords);
                        const float &value = cost_function ? cost_function(cost) : cost;

                        // vector.color = getRainbowColor(value);
                        // vector.pose = mesh_map::calculatePoseFromDirection(
                        //    center, dir_opt.get(), face_normals[fH]);

                        auto u = center;
                        auto v = u + dir_opt.get() * 0.1;

                        if (!std::isfinite(u.x) || !std::isfinite(u.y) || !std::isfinite(u.z) || !std::isfinite(v.x) ||
                            !std::isfinite(v.y) || !std::isfinite(v.z)) {
                            continue;
                        }

                        // vector_field.header.seq = cnt;
                        // vector_field.id = cnt++;
                        vector_field.points.push_back(toPoint(u));
                        vector_field.points.push_back(toPoint(v));

                        std_msgs::ColorRGBA color = getRainbowColor(value);
                        vector_field.colors.push_back(color);
                        vector_field.colors.push_back(color);
                    } else {
                        ROS_ERROR_STREAM_THROTTLE(0.3, "Could not compute the direction!");
                    }
                } else {
                    ROS_ERROR_STREAM_THROTTLE(0.3, "Could not compute the barycentric coords!");
                }
            }
        }
        vector_field_pub.publish(vector_field);
        ROS_INFO_STREAM("Published vector field \"" << name << "\" with " << cnt << " elements.");
    }

    bool MeshMap::inTriangle(const Vector &pos, const lvr2::FaceHandle &face, const float &dist) {
        const auto &vertices = mesh_ptr->getVerticesOfFace(face);
        return mesh_map::inTriangle(pos, mesh_ptr->getVertexPosition(vertices[0]),
                                    mesh_ptr->getVertexPosition(vertices[1]),
                                    mesh_ptr->getVertexPosition(vertices[2]), dist, 0.0001);
    }

    boost::optional <std::tuple<lvr2::FaceHandle, std::array < Vector, 3>, std::array<float, 3>>>

    MeshMap::searchNeighbourFaces(
            const Vector &pos, const lvr2::FaceHandle &face,
            const float &max_radius, const float &max_dist) {
        std::list <lvr2::FaceHandle> possible_faces;
        possible_faces.push_back(face);
        std::list<lvr2::FaceHandle>::iterator face_iter = possible_faces.begin();

        Vector center(0, 0, 0);
        const auto &start_vertices = mesh_ptr->getVertexPositionsOfFace(face);
        for (auto vertex: start_vertices) {
            center += vertex;
        }
        center /= 3;

        float vertex_center_max = 0;
        for (auto vertex: start_vertices) {
            vertex_center_max = std::max(vertex_center_max, vertex.distance(center));
        }

        float ext_radius = max_radius + vertex_center_max;
        float max_radius_sq = ext_radius * ext_radius;

        lvr2::SparseFaceMap<bool> in_list_map;
        in_list_map.insert(face, true);

        std::array<float, 3> bary_coords;

        while (possible_faces.end() != face_iter) {
            const auto &vertices = mesh_ptr->getVertexPositionsOfFace(*face_iter);
            float dist;
            if (mesh_map::projectedBarycentricCoords(pos, vertices, bary_coords, dist) && std::fabs(dist) < max_dist) {
                return std::make_tuple(*face_iter, vertices, bary_coords);
            } else {
                const auto &vertices = mesh_ptr->getVerticesOfFace(*face_iter);
                for (auto vertex: vertices) {
                    if (center.distance2(mesh_ptr->getVertexPosition(vertex)) < max_radius_sq) {
                        try {
                            const auto &nn_faces = mesh_ptr->getFacesOfVertex(vertex);
                            for (auto nn_face: nn_faces) {
                                if (!in_list_map.containsKey(nn_face)) {
                                    possible_faces.push_back(nn_face);
                                    in_list_map.insert(nn_face, true);
                                }
                            }
                        }
                        catch (lvr2::PanicException exception) {
                            // TODO handle case properly
                        }
                    }
                }
                ++face_iter;
            }
        }

        return boost::none;
    }

    bool MeshMap::meshAhead(mesh_map::Vector &pos, lvr2::FaceHandle &face, const float &step_size) {
        std::array<float, 3> bary_coords;
        float dist;
        // get barycentric coordinates of the current face or the next neighbour face
        if (mesh_map::projectedBarycentricCoords(pos, mesh_ptr->getVertexPositionsOfFace(face), bary_coords, dist)) {

        } else if (auto search_res_opt = searchNeighbourFaces(pos, face, step_size, 0.4)) {
            auto search_res = *search_res_opt;
            face = std::get<0>(search_res);
            std::array<Vector, 3> vertices = std::get<1>(search_res);
            bary_coords = std::get<2>(search_res);

            // project position onto surface
            pos = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
        } else {
            return false;
        }
        const auto &opt_dir = directionAtPosition(vector_map, mesh_ptr->getVerticesOfFace(face), bary_coords);
        if (opt_dir) {
            Vector dir = opt_dir.get().normalized();
            std::array<lvr2::VertexHandle, 3> handels = mesh_ptr->getVerticesOfFace(face);
            // iter over all layer vector fields
            for (auto layer: layers) {
                dir += layer.second->vectorAt(handels, bary_coords);
            }
            dir.normalize();
            pos += dir * step_size;
            return true;
        }
        return false;
    }

    lvr2::OptionalFaceHandle MeshMap::getContainingFace(Vector &position, const float &max_dist) {
        auto search_result = searchContainingFace(position, max_dist);
        if (search_result)
            return std::get<0>(*search_result);
        return lvr2::OptionalFaceHandle();
    }

    boost::optional <std::tuple<lvr2::FaceHandle, std::array < mesh_map::Vector, 3>,
    std::array<float, 3>>>
    MeshMap::searchContainingFace(
            Vector
    &position,
    const float &max_dist
    ) {
    if (
    auto vH_opt = getNearestVertexHandle(position)
    ) {
    auto vH = vH_opt.unwrap();
    float min_triangle_position_distance = std::numeric_limits<float>::max();
    std::array<Vector, 3> vertices;
    std::array<float, 3> bary_coords;
    lvr2::OptionalFaceHandle opt_fH;
    for (
    auto fH
    : mesh_ptr->
    getFacesOfVertex(vH)
    ) {
    const auto &tmp_vertices = mesh_ptr->getVertexPositionsOfFace(fH);
    float dist = 0;
    std::array<float, 3> tmp_bary_coords;
    if (
    mesh_map::projectedBarycentricCoords(position, vertices, tmp_bary_coords, dist
    )
    &&
    std::fabs(dist)
    < max_dist) {
    return
    std::make_tuple(fH, tmp_vertices, tmp_bary_coords
    );
}

float triangle_dist = 0;
triangle_dist += (vertices[0] - position).

length2();

triangle_dist += (vertices[1] - position).

length2();

triangle_dist += (vertices[2] - position).

length2();

if (triangle_dist<min_triangle_position_distance) {
min_triangle_position_distance = triangle_dist;
opt_fH = fH;
vertices = tmp_vertices;
bary_coords = tmp_bary_coords;
}
}
if (opt_fH) {
return
std::make_tuple(opt_fH
.

unwrap(), vertices, bary_coords

);
}
ROS_ERROR_STREAM("No containing face found!");
return
boost::none;
}
ROS_FATAL_STREAM("Could not find the nearest vertex");
return
boost::none;
}

lvr2::OptionalVertexHandle MeshMap::getNearestVertexHandle(const Vector &pos) {
    float querry_point[3] = {pos.x, pos.y, pos.z};
    size_t ret_index;
    float out_dist_sqr;
    size_t num_results = kd_tree_ptr->knnSearch(&querry_point[0], 1, &ret_index, &out_dist_sqr);
    return num_results == 0 ? lvr2::OptionalVertexHandle() : lvr2::VertexHandle(ret_index);
}

inline const geometry_msgs::Point MeshMap::toPoint(const Vector &vec) {
    geometry_msgs::Point p;
    p.x = vec.x;
    p.y = vec.y;
    p.z = vec.z;
    return p;
}

constexpr float kEpsilon = 1e-8;

bool MeshMap::projectedBarycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle,
                                         std::array<float, 3> &barycentric_coords, float &dist) {
    const auto &face = mesh_ptr->getVertexPositionsOfFace(triangle);
    return mesh_map::projectedBarycentricCoords(p, face, barycentric_coords, dist);
}

mesh_map::AbstractLayer::Ptr MeshMap::layer(const std::string &layer_name) {
    return layer_names[layer_name];
}

bool MeshMap::barycentricCoords(const Vector &p, const lvr2::FaceHandle &triangle, float &u, float &v, float &w) {
    const auto &face = mesh_ptr->getVertexPositionsOfFace(triangle);
    return mesh_map::barycentricCoords(p, face[0], face[1], face[2], u, v, w);
}

bool MeshMap::rayTriangleIntersect(const Vector &orig, const Vector &dir, const Vector &v0, const Vector &v1,
                                   const Vector &v2, float &t, float &u, float &v, Vector &p) {
    // compute plane's normal
    Vector v0v1 = v1 - v0;
    Vector v0v2 = v2 - v0;

    // no need to normalize
    Vector N = v0v1.cross(v0v2);  // N
    float denom = N.dot(N);

    // Step 1: finding P

    // check if ray and plane are parallel ?
    float NdotRayDirection = N.dot(dir);
    if (fabs(NdotRayDirection) < kEpsilon)  // almost 0
        return false;                         // they are parallel so they don't intersect !

    // compute d parameter using equation 2
    float d = N.dot(v0);

    // compute t (equation 3)
    t = (N.dot(orig) + d) / NdotRayDirection;

    // check if the triangle is in behind the ray
    // if (t < 0) return false; // the triangle is behind

    // compute the intersection point using equation 1
    p = orig + dir * t;

    // Step 2: inside-outside test
    Vector C;  // vector perpendicular to triangle's plane

    // edge 0
    Vector edge0 = v1 - v0;
    Vector vp0 = p - v0;
    C = edge0.cross(vp0);
    if (N.dot(C) < 0)
        return false;  // P is on the right side

    // edge 1
    Vector edge1 = v2 - v1;
    Vector vp1 = p - v1;
    C = edge1.cross(vp1);
    if ((u = N.dot(C)) < 0)
        return false;  // P is on the right side

    // edge 2
    Vector edge2 = v0 - v2;
    Vector vp2 = p - v2;
    C = edge2.cross(vp2);
    if ((v = N.dot(C)) < 0)
        return false;  // P is on the right side;

    u /= denom;
    v /= denom;

    return true;  // this ray hits the triangle
}

bool MeshMap::resetLayers() {
    return true;  // TODO implement
}

void MeshMap::publishCostLayers() {
    ROS_INFO_STREAM("Start publishing");
    for (auto &layer: layers) {
        ROS_INFO_STREAM("Layer \"" << layer.first << "\" try to publish!");


        mesh_msgs_conversions::toVertexCostsStamped(layer.second->costs(), mesh_ptr->numVertices(),
                                                    layer.second->defaultValue(), layer.first, global_frame,
                                                    uuid_str);
        vertex_costs_pub.publish(
                mesh_msgs_conversions::toVertexCostsStamped(layer.second->costs(), mesh_ptr->numVertices(),
                                                            layer.second->defaultValue(), layer.first,
                                                            global_frame,
                                                            uuid_str));

    }


    vertex_costs_pub.publish(
            mesh_msgs_conversions::toVertexCostsStamped(vertex_costs, "Combined Costs", global_frame, uuid_str));

}

void MeshMap::publishVertexCosts(const lvr2::VertexMap<float> &costs, const std::string &name) {
    vertex_costs_pub.publish(
            mesh_msgs_conversions::toVertexCostsStamped(costs, mesh_ptr->numVertices(), 0, name, global_frame,
                                                        uuid_str));
}

void MeshMap::publishVertexColors() {
    using VertexColorMapOpt = lvr2::DenseVertexMapOptional <std::array<uint8_t, 3>>;

    using VertexColorMap = lvr2::DenseVertexMap <std::array<uint8_t, 3>>;

    VertexColorMapOpt vertex_colors_opt = this->mesh_io_ptr->getDenseAttributeMap<VertexColorMap>("vertex_colors");
    if (vertex_colors_opt) {
        const VertexColorMap colors = vertex_colors_opt.get();
        mesh_msgs::MeshVertexColorsStamped msg;
        msg.header.frame_id = mapFrame();
        msg.header.stamp = ros::Time::now();
        msg.uuid = uuid_str;
        msg.mesh_vertex_colors.vertex_colors.reserve(colors.numValues());
        for (auto vH: colors) {
            std_msgs::ColorRGBA color_rgba;
            const auto &color_array = colors[vH];
            color_rgba.a = 1;
            color_rgba.r = color_array[0] / 255.0;
            color_rgba.g = color_array[1] / 255.0;
            color_rgba.b = color_array[2] / 255.0;
            msg.mesh_vertex_colors.vertex_colors.push_back(color_rgba);
        }
        this->vertex_colors_pub.publish(msg);
    }

}

void MeshMap::reconfigureCallback(mesh_map::MeshMapConfig &cfg, uint32_t level) {
    ROS_INFO_STREAM("Dynamic reconfigure callback...");
    if (first_config) {
        config = cfg;
        first_config = false;
        return;
    }

    if (!first_config && map_loaded) {
        if (cfg.cost_limit != config.cost_limit) {
            combineVertexCosts();
        }

        config = cfg;
    }
}

const std::string MeshMap::getGlobalFrameID() {
    return global_frame;
}


void MeshMap::publishSpeedoverAllVertex() {
    float softcap = config.softcap;
    float threshold = config.threshouldSpeed;
    float min = config.minDinstanceSpeed;
    float multi = 2;
    float neuspeed = 0;
    std::ofstream out;
    out.open("/home/lukas/newtest/speedpub/ak.txt", std::ios::app);


    if (vertex_costs.numValues() == 0) {
        speed = 0;
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed;
        ROS_INFO_STREAM("The calculated speed suggestion in percent is 0%, because the created mesh has no vertices");
        speed_pub.publish(speed_msg);
    } else {
        speed = 0;
        float divider = 0;
        if (result == std::numeric_limits<float>::infinity() || result == -std::numeric_limits<float>::infinity()) {
            ROS_INFO("The calculated speed suggestion in percent is 0% because of an object in front of the roboter");
        }

        for (int i = 0; i < vertex_costs.numValues() && result != std::numeric_limits<float>::infinity(); i++) {
            lvr2::VertexHandle vh(i);
            lvr2::BaseVector<float> point = mesh_ptr->getVertexPosition(vh);


            float distance = point.x;
            if (distance > min) {
                if (vertex_costs[vh] == std::numeric_limits<float>::infinity() ||
                    vertex_costs[vh] == -(std::numeric_limits<float>::infinity())) {
                    if (lethals.find(vh) != lethals.end()) {
                        if (distance >= softcap && distance < threshold) {
                            result = penalty * (1 - (distance / threshold));
                            divider = (1 - (distance / threshold));
                        } else if (distance < softcap) {
                            result += vertex_costs[vh];
                            divider = 1;
                        }
                    }
                } else if (distance < threshold) {
                    result = vertex_costs[vh] * (1 - (distance / threshold));
                    divider = (1 - (distance / threshold));
                }

            }
        }

        result /= divider;

        speed = 1 - (multi * (result));
        if (speed == -std::numeric_limits<float>::infinity() || isnan(speed)) {
            speed = 0;
        }
        average_filter_for_speed();
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed;
        ROS_INFO("The calculated speed suggestion in percent is %f", (speed * 100), "%");
        speed_pub.publish(speed_msg);
        out << speed << std::endl;

    }


}

bool MeshMap::getsubscribe() {
    return this->subscribe;
}

void MeshMap::median_filter_for_speed() {
    if (last_speed.size() > 9) {
        last_speed[last_add] = speed;
        last_add++;

    } else {
        last_speed.push_back(speed);
        last_add++;
    }
    if (last_add == 10) {
        last_add = 0;
    }


    if (speed != 0) {
        std::vector<float> order = last_speed;
        std::sort(order.begin(), order.end());
        if (order.size() % 2 == 0) {
            speed = (order[order.size() / 2] + order[(order.size() / 2) - 1]) / 2;
        } else {
            speed = order[order.size() / 2];
        }
    }


}

void MeshMap::average_filter_for_speed() {
    if (last_speed.size() > 9) {
        last_speed[last_add] = speed;
        last_add++;

    } else {
        last_speed.push_back(speed);
        last_add++;
    }
    if (last_add == 10) {
        last_add = 0;
    }


    if (speed != 0) {
        float sum = 0;
        for (int i = 0; i < last_speed.size(); i++) {
            sum += last_speed[i];
        }
        speed = sum / last_speed.size();
    }


}


} /* namespace mesh_map */
