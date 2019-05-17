/*
 *  Copyright 2019, Sabrina Frohn
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
 *    Sabrina Frohn <sfrohn@uni-osnabrueck.de>
 *
 */

#ifndef MESH_NAVIGATION__MESH_CONTROLLER_H
#define MESH_NAVIGATION__MESH_CONTROLLER_H

#include <mbf_mesh_core/mesh_controller.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
// #include <mesh_controller/MeshControllerConfig.h>

namespace mesh_controller{

    class MeshController : public mbf_mesh_core::MeshController{

        public:

            typedef boost::shared_ptr<mesh_controller::MeshController> Ptr;

            MeshController();

            /**
             * @brief Destructor
             */
            virtual ~MeshController();

            /**
             * @brief Given the current position, orientation, and velocity of the robot,
             * compute velocity commands to send to the base.
             * @param pose The current pose of the robot.
             * @param velocity The current velocity of the robot.
             * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
             * @param message Optional more detailed outcome as a string
             * @return Result code as described on ExePath action result:
             *         SUCCESS         = 0
             *         1..9 are reserved as plugin specific non-error results
             *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
             *         CANCELED        = 101
             *         NO_VALID_CMD    = 102
             *         PAT_EXCEEDED    = 103
             *         COLLISION       = 104
             *         OSCILLATION     = 105
             *         ROBOT_STUCK     = 106
             *         MISSED_GOAL     = 107
             *         MISSED_PATH     = 108
             *         BLOCKED_PATH    = 109
             *         INVALID_PATH    = 110
             *         TF_ERROR        = 111
             *         NOT_INITIALIZED = 112
             *         INVALID_PLUGIN  = 113
             *         INTERNAL_ERROR  = 114
             *         121..149 are reserved as plugin specific errors
             */
            virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message);

            /**
             * @brief Check if the goal pose has been achieved by the local planner
             * @param pose The current pose of the robot.
             * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
             * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
             * @return True if achieved, false otherwise
             */
            virtual bool isGoalReached(double dist_tolerance, double angle_tolerance);

            /**
             * @brief Set the plan that the local planner is following
             * @param plan The plan to pass to the local planner
             * @return True if the plan was updated successfully, false otherwise
             */
            virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

            /**
             * @brief Requests the planner to cancel, e.g. if it takes too much time.
             * @return True if a cancel has been successfully requested, false if not implemented.
             */
            virtual bool cancel();

            /**
             * Results in a slow increase of velocity in the beginning. Can be modulated via fading variable.
             * @return factor by which to multiply the velocity for smooth start
             */
            float startVelocityFactor();

            /**
             * Results in a slow decrease of velocity towards the goal. Can be modulated via fading varibale.
             * @return factor by which to multiply the velocity for smooth stop
             */
            float endVelocityFactor(;

            /**
             * Transforms a PoseStamped into a mesh_map Vector
             * @param pose      any geometry_msgs PoseStamped
             * @return          mesh_map Vector
             */
            mesh_map::Vector poseToVector(const geometry_msgs::PoseStamped& pose);

            /**
             * Calculates the (smaller) angle between two vectors
             * @param pos       mesh_map Vector of the current robot heading
             * @param plan      mesh_map Vector of the supposed (planned) heading
             * @param leftRight is set to -1 if a left turn has to be made, or +1 for right turn
             * @return          (smaller) angle between the vectors
             */
            float angleBetweenVectors(mesh_map::Vector pos, mesh_map::Vector plan);

            /**
             * A *tangens* function to return a phased response value given a value
             * @param max_hight     maximum value that will be returned
             * @param max_width     range that the given value can take before the maximum is returned
             * @param value         value on x axis
             * @return              value on y axis that corresponds to input value
             */
            float tanValue(float max_hight, float max_width, float value);

            /**
             * A *normal distribution / gaussian* function to return a phased response value given a value
             * @param max_hight     maximum value that will be returned
             * @param max_width     range that the given value can take before the maximum is returned
             * @param value         value on x axis
             * @return              value on y axis that corresponds to input value
             */
            float gaussValue(float max_hight, float max_width, float value);

            float euclideanDistance(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& plan_position);

            /**
             * @brief Calculates the Euclidean Distance between the current robot pose and the next [20] samples of the
             *          plan to find the closest part of the plan.
             * @param pose The current pose of the robot.
             */
            void updatePlanPos(const geometry_msgs::PoseStamped& pose, float velocity);

            /**
             * Checks the difficulty of the next vertex / vertices (depending on velocity) to calculate a factor by which to change the velocity
             * @param pose          current position of the robot
             * @param velocity      speed of the robot
             * @return              factor to increase/decrease/keep the robot linear velocity between -1 and 1
             */
            float lookAhead(const geometry_msgs::PoseStamped& pose, float velocity);

            /**
             * Calculates the cost of the vertex of the current pose
             * @param pose  Current position of the robot
             * @return      Cost of the vertex
             */
            float cost(const geometry_msgs::PoseStamped& pose);

            /**
             * Calculates the cost of the vertex of the current pose.
             * @param pose_vec  Pointer to the vector of the current pose
             * @param face      pointer to the face handle of the vector
             * @return      Cost of the vertex
             */
            float cost(mesh_map::Vector &pose_vec, const lvr2::FaceHandle &face);

            /**
             * @brief Checks if the robots' direction is aligned with the path
             * @param current_angle     Current orientation of the robot.
             * @param goal_angle        Orientation of  path.
             * @param angle_tolerance   The angle tolerance in which the current pose will be partly accepted as aligned with path
             * @return true if robot is aligned; false otherwise
            */
            bool align(float current_angle, float goal_angle, float angle_tolerance);

            /**
             * @brief           PID controller: compares the actual robot pose with the desired one and calculates a
             *                  calculating term
             * @param setpoint  pose of the desired position of the robot
             * @param pv        pose of the actual robot position
             * @return          correcting term
             */
            float pidControl(const geometry_msgs::PoseStamped& setpoint, const geometry_msgs::PoseStamped& pv);

            void recordData(const geometry_msgs::PoseStamped& robot_pose);

            virtual bool initialize(
                    const std::string& name,
                    const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                    const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr);

        protected:

        private:
            boost::shared_ptr<mesh_map::MeshMap> mesh;
            vector<geometry_msgs::PoseStamped> current_plan;
            geometry_msgs::PoseStamped goal;
            geometry_msgs::PoseStamped current_position;
            int iter;
            float int_error;
            // loop interval time in sec
            float int_time;
            float prev_error;
            // set true, footprint will be considered in planner
            bool footprint;
            bool haveStartFace;
            lvr2::FaceHandle current_face;
            bool record;
            // angle between pose vector and planned / supposed vector
            float angle;
            // determines how long it takes at the start/end to reach full/zero velocity (btw 0 and 100)
            float fading;
            const float prop_gain = 1.0;
            const float int_gain = 1.0;
            const float deriv_gain = 1.0;
            const float PI = 3.141592;
            const float E = 3.718281;
            const float maximum_velocity = 0.5;

    };

} /* namespace mesh_controller */
#endif /* MESH_NAVIGATION__MESH_CONTROLLER_H */
