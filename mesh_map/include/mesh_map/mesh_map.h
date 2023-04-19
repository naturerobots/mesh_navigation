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

#ifndef MESH_MAP__MESH_MAP_H
#define MESH_MAP__MESH_MAP_H

#include <atomic>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/io/HDF5IO.hpp>
#include <mesh_map/MeshMapConfig.h>
#include <mesh_map/abstract_layer.h>
#include <mesh_msgs/MeshVertexCosts.h>
#include <mesh_msgs/MeshVertexColors.h>
#include <mutex>
#include <pluginlib/class_loader.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>
#include <tuple>
#include "nanoflann.hpp"
#include "nanoflann_mesh_adaptor.h"
#include <sensor_msgs/PointCloud2.h>
#include <organized_fast_mesh_generator.h>
#include <lvr2/geometry/Quaternion.hpp>
namespace mesh_map
{
    class MeshMap
    {
    public:
        typedef boost::shared_ptr<MeshMap> Ptr;

        MeshMap(tf2_ros::Buffer& tf, bool subscribe =false);

        /**
         * @brief Calculate normals and cost values and publishes all as mesh_msgs
         * @return true f the mesh and its attributes have been load successfully.
         */
        bool readMap();

        /**
         * @brief Loads all configures layer plugins
         * @return true if the layer plugins have been load successfully.
         */
        bool loadLayerPlugins();

        /**
         * @brief Initialized all loaded layer plugins
         * @return true if the loaded layer plugins have been initialized successfully.
         */
        bool initLayerPlugins();

        /**
         * @brief Return and optional vertex handle of to the closest vertex to the given position
         * @param pos the search position
         * @return
         */
        lvr2::OptionalVertexHandle getNearestVertexHandle(const mesh_map::Vector& pos);

        /**
         * @brief return true if the given position lies inside the triangle with respect to the given maximum distance.
         * @param pos The query position
         * @param face The triangle to check for
         * @param dist The maximum distance to the triangle
         * @return true if the query position is inside the triangle within the maximum distance
         */
        bool inTriangle(const Vector& pos, const lvr2::FaceHandle& face, const float& dist);

        /**
         * @brief Searches for a triangle which contains the given position with respect to the maximum distance
         * @param position The query position
         * @param max_dist The maximum distance to the triangle
         * @return Optional face handle, the optional is valid if a corresponding triangle has been found.
         */
        lvr2::OptionalFaceHandle getContainingFace(Vector& position, const float& max_dist);

        /**
         * @brief Searches for a triangle which contains the given position with respect to the maximum distance
         * @param position The query position
         * @param max_dist The maximum distance to the triangle
         * @return optional tuple of the corresponding triangle, the triangle's vertices, and barycentric coordinates, if a corresponding triangle has been found.
         */
        boost::optional<std::tuple<lvr2::FaceHandle, std::array<mesh_map::Vector, 3>,
                std::array<float, 3>>> searchContainingFace(Vector& position, const float& max_dist);

        /**
         * @brief reconfigure callback function which is called if a dynamic reconfiguration were triggered.
         */
        void reconfigureCallback(mesh_map::MeshMapConfig& config, uint32_t level);

        /**
         * @brief A method which combines all layer costs with the respective weightings
         */
        void combineVertexCosts();

        /**
         * @brief Computes contours
         * @param contours the vector to bo filled with contours
         * @param min_contour_size The minimum contour size, i.e. the number of vertices per contour.
         */
        void findContours(std::vector<std::vector<lvr2::VertexHandle>>& contours, int min_contour_size);

        /**
         * @brief Publishes the given vertex map as mesh_msgs/VertexCosts, e.g. to visualize these.
         * @param costs The cost map to publish
         * @param name The name of the cost map
         */
        void publishVertexCosts(const lvr2::VertexMap<float>& costs, const std::string& name);

        /**
         * @briefP Publishes the vertex colors if these exists.
         */
        void publishVertexColors();

        /**
         * @brief Publishes all layer costs as mesh_msgs/VertexCosts
         */
        void publishCostLayers();

        /**
         * @brief Computes the projected barycentric coordinates, it implements Heidrich's method
         * See https://www.researchgate.net/publication/220494112_Computing_the_Barycentric_Coordinates_of_a_Projected_Point
         * @param p the query position
         * @param triangle the corresponding triangle
         * @param barycentric_coords The array will be filled with the barycentric coordinates
         * @param dist The distance if the query position to the triangle
         * @return ture if the query position lies inside the triangle
         */
        bool projectedBarycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle,
                                        std::array<float, 3>& barycentric_coords, float& dist);

        /**
         * Computes barycentric coordinates of the given query position and the corresponding triangle
         * @param p The query position
         * @param triangle The corresponding triangle
         * @param u The barycentric coordinate if the first vertex
         * @param v The barycentric coordinate if the second vertex
         * @param w The barycentric coordinate if the third vertex
         * @return true if the query position lies inside the triangle
         */
        bool barycentricCoords(const Vector& p, const lvr2::FaceHandle& triangle, float& u, float& v, float& w);

        /**
         * @brief Callback function which is called from inside a layer plugin if cost values change
         * @param layer_name the name of the layer.
         */
        void layerChanged(const std::string& layer_name);

        /**
         * @brief Compute all contours and returns the corresponding vertices to use these as lethal vertices.
         * @param min_contour_size The minimum contour size, i.e. the number of vertices per contour.
         * @param min_contour_size
         * @param lethals the vector which is filled with contour vertices
         */
        void findLethalByContours(const int& min_contour_size, std::set<lvr2::VertexHandle>& lethals);

        /**
         * @brief Returns the global frame / coordinate system id string
         */
        const std::string getGlobalFrameID();

        /**
         * @brief Checks if the given vertex handle corresponds to a lethal / not passable vertex
         */
        inline bool isLethal(const lvr2::VertexHandle& vH);

        /**
         * @brief Converts a vector to a ROS geometry_msgs/Point message
         */
        inline const geometry_msgs::Point toPoint(const Vector& vec);

        /**
         * Computes the direction vector for the given triangle's vertices and barycentric coordinates while using the given vector map
         * @param vector_map The vector map to use
         * @param vertices The triangles vertices
         * @param barycentric_coords The barycentric coordinates of the query position.
         * @return An optional vector of the computed direction. It is valid if a vector has been computed successfully.
         */
        boost::optional<Vector> directionAtPosition(const lvr2::VertexMap<lvr2::BaseVector<float>>& vector_map,
                                                    const std::array<lvr2::VertexHandle, 3>& vertices,
                                                    const std::array<float, 3>& barycentric_coords);

        /**
         * Computes the cost value for the given triangle's vertices and barycentric coordinates while using the given cost map
         * @param costs The cost map to use
         * @param vertices The triangles vertices
         * @param barycentric_coords The barycentric coordinates of the query position.
         * @return A cost value for the given barycentric coordinates.
         */
        float costAtPosition(const lvr2::DenseVertexMap<float>& costs, const std::array<lvr2::VertexHandle, 3>& vertices,
                             const std::array<float, 3>& barycentric_coords);

        /**
         * Computes the cost value for the given triangle's vertices and barycentric coordinates while using the combined cost map
         * @param vertices The triangles vertices
         * @param barycentric_coords The barycentric coordinates of the query position.
         * @return A cost value for the given barycentric coordinates.
         */
        float costAtPosition(const std::array<lvr2::VertexHandle, 3>& vertices,
                             const std::array<float, 3>& barycentric_coords);

        /**
         * Computes the barycentric coordinates of the ray intersection point for the given ray
         * @param orig The ray origin
         * @param dir The ray direction
         * @param v0 The triangle's first vertex
         * @param v1 The triangle's second vertex
         * @param v2 The triangle's third vertex
         * @param t The signed distance to the triangle
         * @param u The first barycentric coordinate
         * @param v The second barycentric coordinate
         * @param p The thrid barycentric coordinate
         * @return True if the intersection point lies inside the triangle
         */
        bool rayTriangleIntersect(const Vector& orig, const Vector& dir, const Vector& v0, const Vector& v1, const Vector& v2,
                                  float& t, float& u, float& v, Vector& p);

        /**
         * @brief Resets all layers.
         * @todo implement
         * @return true if successfully reset
         */
        bool resetLayers();

        /**
         * @brief Returns the stored vector map
         */
        const lvr2::DenseVertexMap<mesh_map::Vector>& getVectorMap()
        {
            return vector_map;
        };

        /**
         * @brief Returns the stored mesh
         */
        const lvr2::HalfEdgeMesh<Vector>& mesh()
        {
            return *mesh_ptr;
        }

        /**
         * @brief Returns the stored combined costs
         */
        const lvr2::DenseVertexMap<float>& vertexCosts()
        {
            return vertex_costs;
        }

        /**
         * @brief Returns the map frame / coordinate system id
         */
        const std::string& mapFrame()
        {
            return global_frame;
        }

        /**
         * @brief Returns the mesh's triangle normals
         */
        const lvr2::DenseFaceMap<Normal>& faceNormals()
        {
            return face_normals;
        }

        /**
         * @brief Returns the mesh's vertex normals
         */
        const lvr2::DenseVertexMap<Normal>& vertexNormals()
        {
            return vertex_normals;
        }

        /**
         * @brief Returns the mesh's edge weights
         */
        const lvr2::DenseEdgeMap<float>& edgeWeights()
        {
            return edge_weights;
        }

        /**
         * @brief Returns the mesh's vertex distances
         */
        const lvr2::DenseEdgeMap<float>& edgeDistances()
        {
            return edge_distances;
        }

        /**
         * Searches in the surrounding triangles for the triangle in which the given
         * position lies.
         * @param pose_vec    Vector of the position which searched for
         * @param face        Face handle from which search begins
         * @param max_radius  The radius in which the controller searches for a consecutive face
         * @param max_dist    The maximum distance to the given face vertices
         * @return            Optional of (face handle, vertices, and barycentric coord) tuple
         */
        boost::optional<std::tuple<lvr2::FaceHandle, std::array<Vector, 3>, std::array<float, 3>>>
        searchNeighbourFaces(const Vector& pos, const lvr2::FaceHandle& face,
                             const float& max_radius, const float& max_dist);

        /**
         * Finds the next position given a position vector and its corresponding face
         * handle by following the direction For: look ahead when using mesh gradient
         * @param vec   direction vector from which the next step vector is calculated
         * @param face  face of the direction vector
         * @param step_width The step length to go ahead on the mesh surface
         * @return      new vector (also updates the ahead_face handle to correspond
         * to the new vector)
         */
        bool meshAhead(Vector& vec, lvr2::FaceHandle& face, const float& step_width);

        /**
         * @brief Stores the given vector map
         */
        void setVectorMap(lvr2::DenseVertexMap<mesh_map::Vector>& vector_map);

        /**
         * @brief Publishes a position as marker. Used for debug purposes.
         * @param pos The position to publish as marker
         * @param color The marker's color
         * @param name The marker's name
         */
        void publishDebugPoint(const Vector pos, const std_msgs::ColorRGBA& color, const std::string& name);

        /**
         * @brief Publishes a triangle as marker. Used for debug purposes.
         * @param face_handle The face handle for the triangle
         * @param color The marker's color
         * @param name The marker's name
         */
        void publishDebugFace(const lvr2::FaceHandle& face_handle, const std_msgs::ColorRGBA& color, const std::string& name);

        /**
         * @brief Publishes a vector field as visualisation_msgs/Marker
         * @param name The marker's name
         * @param vector_map The vector field to publish
         * @param publish_face_vectors Enables to publish an additional vertex for the triangle's center.
         */
        void publishVectorField(const std::string& name, const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                                const bool publish_face_vectors = false);

        /**
         * @brief Publishes a vector field as visualisation_msgs/Marker
         * @param name The marker's name
         * @param vector_map The vector field to publish
         * @param values The vertex cost values
         * @param cost_function A cost function to compute costs inside a triangle
         * @param publish_face_vectors Enables to publish an additional vertex for the triangle's center.
         */
        void publishVectorField(const std::string& name, const lvr2::DenseVertexMap<lvr2::BaseVector<float>>& vector_map,
                                const lvr2::DenseVertexMap<float>& values,
                                const std::function<float(float)>& cost_function = {},
                                const bool publish_face_vectors = false);

        /**
         * @brief Publishes the vector field as visualisation_msgs/Marker
         */
        void publishCombinedVectorField();

        /**
         * @brief returns a shared pointer to the specified layer
         */
        mesh_map::AbstractLayer::Ptr layer(const std::string& layer_name);

        /**
         * @brief create a Organized Fast Mesh and write it to mesh_ptr
         * @param cloud
         */
        void createOFM(const sensor_msgs::PointCloud2::ConstPtr &cloud);

        /**
         *  @brief calculate and publish a speed estimation
         */
        void publishSpeedoverAllVertex();
        /**
         * Retrun the subscribe value
         * @return subscribe
         */
        bool getsubscribe();
        std::shared_ptr<lvr2::HalfEdgeMesh<lvr2::BaseVector<float>>> mesh_ptr;
        std::shared_ptr<lvr2::AttributeMeshIOBase> mesh_io_ptr;

        lvr2::DenseVertexMap<bool> invalid;

    private:
        bool isInsideBox(lvr2::BaseVector<float> p, lvr2::BaseVector<float>* vertices);

        void checkleathleObjectsbetweenWheels(lvr2::PointBuffer &cloudBuffer);

        bool subscribe;

        ros::Subscriber cloud_sub_;
        ros::Publisher mesh_pub_;
        std::string point_cloud;
        //! plugin class loader for for the layer plugins
        pluginlib::ClassLoader<mesh_map::AbstractLayer> layer_loader;

        //! mapping from name to layer instance
        std::map<std::string, mesh_map::AbstractLayer::Ptr> layer_names;

        //! vector of name and layer instances
        std::vector<std::pair<std::string, mesh_map::AbstractLayer::Ptr>> layers;

        //! each layer maps to a set of impassable indices
        std::map<std::string, std::set<lvr2::VertexHandle>> lethal_indices;

        //! all impassable vertices
        std::set<lvr2::VertexHandle> lethals;

        //! global frame / coordinate system id
        std::string global_frame;

        //! server url
        std::string srv_url;

        //! login username to connect to the server
        std::string srv_username;

        //! login password to connect to the server
        std::string srv_password;

        std::string mesh_layer;


        float result;
        float min_roughness;
        float max_roughness;
        float min_height_diff;
        float max_height_diff;
        float bb_min_x;
        float bb_min_y;
        float bb_min_z;
        float bb_max_x;
        float bb_max_y;
        float bb_max_z;

        std::string mesh_file;
        std::string mesh_part;

        //! combined layer costs
        lvr2::DenseVertexMap<float> vertex_costs;

        //! stored vector map to share between planner and controller
        lvr2::DenseVertexMap<mesh_map::Vector> vector_map;

        //! vertex distance for each edge
        lvr2::DenseEdgeMap<float> edge_distances;

        //! edge weights
        lvr2::DenseEdgeMap<float> edge_weights;

        //! triangle normals
        lvr2::DenseFaceMap<Normal> face_normals;

        //! vertex normals
        lvr2::DenseVertexMap<Normal> vertex_normals;

        //! publisher for vertex costs
        ros::Publisher vertex_costs_pub;

        //! publisher for speed
        ros::Publisher speed_pub;

        //! publisher for vertex colors
        ros::Publisher vertex_colors_pub;

        //! publisher for the mesh geometry
        ros::Publisher mesh_geometry_pub;

        //! publisher for the debug markers
        ros::Publisher marker_pub;

        //! publisher for the stored vector field
        ros::Publisher vector_field_pub;

        //! shared pointer to dynamic reconfigure server
        boost::shared_ptr<dynamic_reconfigure::Server<mesh_map::MeshMapConfig>> reconfigure_server_ptr;

        //! dynamic reconfigure callback function binding
        dynamic_reconfigure::Server<mesh_map::MeshMapConfig>::CallbackType config_callback;

        //! first reconfigure call
        bool first_config;

        //! indicates whether the map has been loaded
        bool map_loaded;

        //! current mesh map configuration
        MeshMapConfig config;

        //! private node handle within the mesh map namespace
        ros::NodeHandle private_nh;

        //! transformation buffer
        tf2_ros::Buffer& tf_buffer;

        //! uuid for the load mesh map
        std::string uuid_str;

        //! layer mutex to handle simultaneous layer changes
        std::mutex layer_mtx;

        //! k-d tree type for 3D with a custom mesh adaptor
        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<float, NanoFlannMeshAdaptor>,
                NanoFlannMeshAdaptor, 3> KDTree;

        //! k-d tree nano flann mesh adaptor to access mesh data
        std::unique_ptr<NanoFlannMeshAdaptor> adaptor_ptr;

        //! k-d tree to query mesh vertices in logarithmic time
        std::unique_ptr<KDTree> kd_tree_ptr;
        lvr2::Quaternion<lvr2::BaseVector<float>> transform;
        lvr2::Quaternion<lvr2::BaseVector<float>> transform_to_base;

        lvr2::BaseVector<float> left_wheel;
        lvr2::BaseVector<float> right_wheel;
        lvr2::BaseVector<float> width_of_intresst;
        lvr2::BaseVector<float> depth_of_intresst;
        lvr2::BaseVector<float> hight_of_intresst;
        lvr2::BaseVector<float> length_of_intresst;
        lvr2::BaseVector<float> area_of_interesst_left[8];
        lvr2::BaseVector<float> area_of_interesst_right[8];
        lvr2::BaseVector<float> roboter_polyeder[8];
        std::vector<float> last_speed;
        int last_add =0;
        float  penalty;
        int i=0;
        float speed =0;

        lvr2::Matrix4< lvr2::BaseVector<float>> matrixTransform;
        float divider =0;

    };

} /* namespace mesh_map */

#endif  // MESH_NAVIGATION__MESH_MAP_H