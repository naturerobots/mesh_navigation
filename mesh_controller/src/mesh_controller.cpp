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

#include <pluginlib/class_list_macros.h>
#include <mesh_controller/mesh_controller.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <lvr_ros/colors.h>

PLUGINLIB_EXPORT_CLASS(mesh_controller::MeshController, mbf_mesh_core::MeshController);

namespace mesh_controller{

    MeshController::MeshController()
    {

    }

    MeshController::~MeshController()
    {

    }

    uint32_t MeshController::computeVelocityCommands(
        const geometry_msgs::PoseStamped& pose,
        const geometry_msgs::TwistStamped& velocity,
        geometry_msgs::TwistStamped &cmd_vel,
        std::string &message
        )
    {
        // start an iterator to go through plan vector
        int iter = 0;

        // repeatedly check if goal is reached

        while(!isGoalReached(0.1, 0.1)){
            // set current_position as first pose from plan vector
            current_position = current_plan[iter];

            ROS_INFO("start movement");

            // get yaw from quaternion
            float current_angle = quaternionToYaw(pose);
            float goal_angle = quaternionToYaw(current_position);

            // align robot angle with paths angle until certain tolerance is reached
            if (!align(current_angle, goal_angle, 0.1)) {
                // TODO check if angular momentum is correct
                if(current_angle < goal_angle){
                    cmd_vel.twist.angular.z = 0.1;
                } else {
                    cmd_vel.twist.angular.z = -0.1;
                }
            }
            else{
                // move forward
                // TODO use look ahead function to determine new velocity
                cmd_vel.twist.linear.x = 0.2;
                iter += 1;
            }
        }
        stopRobot(cmd_vel);

        return mbf_msgs::GetPathResult::SUCCESS;
    }

    bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        goal = current_plan.back();
        geometry_msgs::PoseStamped pose = current_position;

        // get yaw from quaternion
        float current_angle = quaternionToYaw(pose);
        float goal_angle = quaternionToYaw(goal);

        // robot point in 3D space
        float current_x = pose.pose.position.x;
        float current_y = pose.pose.position.y;
        float current_z = pose.pose.position.z;

        // goal point in 3D space
        float goal_x = goal.pose.position.x;
        float goal_y = goal.pose.position.y;
        float goal_z = goal.pose.position.z;


        // test if robot is within tolerable distance to goal
        if (current_angle <= goal_angle + angle_tolerance && current_angle >= goal_angle - angle_tolerance ||
            current_x <= goal_x + dist_tolerance && current_x >= goal_x - dist_tolerance ||
            current_y <= goal_y + dist_tolerance && current_y >= goal_y - dist_tolerance ||
            current_z <= goal_z + dist_tolerance && current_z >= goal_z - dist_tolerance){
            return true;
        } else {
            return false;
        }
    }

    bool  MeshController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!plan.empty()) {
            current_plan = plan;
            return true;
        }
        else {
            return false;
        }
    }

    bool  MeshController::cancel()
    {
        return false;
    }



    float MeshController::quaternionToYaw(const geometry_msgs::PoseStamped& pose){
        // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        float a = pose.pose.orientation.x;
        float b = pose.pose.orientation.z;
        float mag = sqrtf(a*a + b*b);
        a /= mag;
        return 2*acosf(a);

    }

    bool MeshController::stopRobot(geometry_msgs::TwistStamped &cmd_vel){
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel.twist.linear.x = 0.0;
        return true;
    }

    float MeshController::euclideanDistance(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& plan_position){
        // https://en.wikipedia.org/wiki/Euclidean_distance

        float px = pose.pose.position.x;
        float py = pose.pose.position.y;
        float pz = pose.pose.position.z;

        float cx = plan_position.pose.position.x;
        float cy = plan_position.pose.position.y;
        float cz = plan_position.pose.position.z;

        float power = 2.0;
        float dist = sqrtf((pow((px-cx),power) + pow((py-cy),power) + pow((pz-cz),power)));
        return dist;
    }

    /**
     * Checks if the next vertex is the goal.
     * @return true, if it is the goal; false otherwise
     */
    /*
   bool MeshController::goalAhead(current_vertex, )
   {
       if(next_vertex == goal_vertex){
           return true
       }
       return false
   }
   */

    /**
     * Checks the difficulty of the next vertex / vertices (depending on velocity) to set the speed accordingly
     * @param name
     * @param tf_ptr
     * @param mesh_map_ptr
     * @return indentifier how speed shall be handled
     *          0 - same speed
     *          1 - increase speed
     *          2 - reduce speed
     *          3 - goal within reach
     *          4 - calculation failed
     */
     /*
     int MeshController::lookAhead(current_cost, index, speed)
     {
        int amount = 0;
        if (speed <= 0.1){
            amount = 3;
        } else if (speed <= 0.2){
            amount = 6;
        } else if (speed <= 0.3) {
            amount = 9;
        } else {
            amount = 12
        }

        // checks if goal is within reach
        if ( (index + amount) > current_plan.size()) {
            return 3;
        }

        int all_vertices = 0;

        for(int i = 0; i < amount; i++){
            // sum up all vertex costs
            all_vertices += current_plan[index + i];
        }

        // get average vertex costs
        int av_cost = all_vertices / amount;

        // TODO think about making more phased (gestaffelte) decisions
        if (current_cost == av_cost){
            return 0;
        } else if (current_cost > av_cost) {
            return 1;
        } else if (current_cost < av_cost) {
            return 2;
        } else {
            return 4;
        }
    }
    */


    bool MeshController::align(float current_angle, float goal_angle, float angle_tolerance)
    {
        // test if robot is within tolerable distance to goal
        if (current_angle <= goal_angle + angle_tolerance && current_angle >= goal_angle - angle_tolerance){
            return true;
        } else {
            return false;
        }
    }

    bool  MeshController::initialize(
        const std::string& name,
        const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
        const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
     {
        return true;
    }
}
