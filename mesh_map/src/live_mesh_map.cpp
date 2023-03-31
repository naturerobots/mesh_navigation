//
// Created by lukas on 29.03.23.
//

#include "mesh_map/live_mesh_map.h"
#include <lvr_ros/conversions.h>
#include "std_msgs/Float64.h"
#include <mesh_map/mesh_map.h>
#include <mesh_map/util.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs_conversions/conversions.h>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/algorithm/GeometryAlgorithms.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <organized_fast_mesh_generator.h>
namespace live_mesh_map {
    LiveMeshMap::LiveMeshMap(tf2_ros::Buffer &tf_listener)
    :MeshMap(tf_listener){
        cloud_sub_ = private_nh.subscribe("/ouster/destaggeredpoints", 100, &LiveMeshMap::createOFM, this);
        speed_pub = private_nh.advertise<std_msgs::Float64>("speed", 1, false);

    }

    void LiveMeshMap::createOFM(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        int rowstep = 1;
        int calstep = 1;
        lvr2::PointBuffer pointBuffer;
        lvr_ros::fromPointCloud2ToPointBuffer(*cloud, pointBuffer);
        OrganizedFastMeshGenerator ofmg = OrganizedFastMeshGenerator(pointBuffer, cloud->height, cloud->width,rowstep,calstep,-0.5,0.5,0.3,0.1, 0.5);
        ofmg.setEdgeThreshold(1);
        lvr2::MeshBufferPtr mesh_buffer_ptr(new lvr2::MeshBuffer);
        mesh_msgs::MeshVertexColorsStamped color_msg;
        ofmg.getMesh(*mesh_buffer_ptr, color_msg);
        mesh_msgs::MeshGeometry mesh_map;
        lvr_ros::fromMeshBufferToMeshGeometryMessage(mesh_buffer_ptr, mesh_map);
        *mesh_ptr = lvr2::HalfEdgeMesh<lvr2::BaseVector<float>>(mesh_buffer_ptr);
        this->readMap();
        this->vertex_colors_pub.publish(color_msg);
        //publishSpeed(5,vec,rowstep,calstep,20);
        publishSpeedoverAllVertex();

    }

    bool LiveMeshMap::readMap() {
        ROS_INFO_STREAM("The mesh has been create successfully with " << mesh_ptr->numVertices() << " vertices and "
                                                                          << mesh_ptr->numFaces() << " faces and "
                                                                          << mesh_ptr->numEdges() << " edges.");

            adaptor_ptr = std::make_unique<mesh_map::NanoFlannMeshAdaptor>(*mesh_ptr);
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
                    std::set<lvr2::VertexHandle> empty;
                    layer_plugin->updateLethal(lethals, empty);
                    layer_plugin->computeLayer(false);
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
    bool LiveMeshMap::initLayerPlugins(bool hasIO){
        MeshMap::initLayerPlugins(hasIO);
    }


    void LiveMeshMap::publishSpeedoverAllVertex(){
        float softcap = 0;
        float threshold = 10;
        float min=0.1;
        int bad =0;
        int nice =0;
        float result =0;
        if (vertex_costs.numValues() ==0){
            float speed = 0;
            std_msgs::Float64 speed_msg;
            speed_msg.data = speed;
            ROS_INFO_STREAM("speed");
            ROS_INFO_STREAM(speed);
            speed_pub.publish(speed_msg);
        }
        else {
            for (int i = 0; i < vertex_costs.numValues(); i++) {
                lvr2::VertexHandle vh(i);
                lvr2::BaseVector<float> point =mesh_ptr->getVertexPosition(vh);
                float distance = sqrt(pow(point.x,2)+pow(abs(point.y)-0.5,2));

                if(distance>min) {
                    if (vertex_costs[vh] == std::numeric_limits<float>::infinity() ||
                        vertex_costs[vh] == -(std::numeric_limits<float>::infinity())) {
                        if (lethals.find(vh) != lethals.end()) {
                            if (distance >= softcap && distance < threshold) {
                                result = (result * distance / threshold) + (10 * (1 - (distance / threshold)));
                            } else if (distance < softcap) {
                                result += vertex_costs[vh];
                            }
                        }
                    } else if (distance < threshold) {
                        result = (result * distance / threshold) + (vertex_costs[vh] * (1 - (distance / threshold)));
                    }
                }
            }

            float speed = 1 - (0.1 * (result));
            if(speed<0){
                speed=0;
            }
            //wilde normierungsaktion
            std_msgs::Float64 speed_msg;
            speed_msg.data = speed;

            ROS_INFO_STREAM(result);
            ROS_INFO_STREAM("speed");
            ROS_INFO_STREAM(speed);
            ROS_INFO_STREAM(nice);
            ROS_INFO_STREAM(bad);
            speed_pub.publish(speed_msg);
        }




    }





}