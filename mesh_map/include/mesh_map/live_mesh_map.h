//
// Created by lukas on 29.03.23.
//

#ifndef SRC_LIVE_MESH_MAP_H
#define SRC_LIVE_MESH_MAP_H

#endif //SRC_LIVE_MESH_MAP_H
#include <lvr2/geometry/BaseVector.hpp>
#include "mesh_map.h"

namespace live_mesh_map {
class LiveMeshMap : public mesh_map::MeshMap {
    public:
        typedef boost::shared_ptr<LiveMeshMap> Ptr;
        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<float,  mesh_map::NanoFlannMeshAdaptor>,
                mesh_map::NanoFlannMeshAdaptor, 3> KDTree;

        LiveMeshMap(tf2_ros::Buffer &tf);

        /**
        * @brief Calculate normals and cost values and publishes all as mesh_msgs
         * * @return true f the mesh and its attributes have been load successfully.
          */
        bool readMap();

        /**
        * @brief Initialized all loaded layer plugins
        * @return true if the loaded layer plugins have been initialized successfully.
        */
        bool initLayerPlugins(bool hasIO = false);

        void publishSpeedoverAllVertex();

        void createOFM(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    private:
        std::shared_ptr<OrganizedFastMeshGenerator> ofmg_ptr;
        lvr2::HalfEdgeMesh<mesh_map::Vector> organizedMesh;
        ros::Subscriber cloud_sub_;
        ros::Publisher mesh_pub_;
        ros::Publisher speed_pub;

};
}