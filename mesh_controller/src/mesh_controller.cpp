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
        ROS_INFO("start movement");
        float new_velocity = 0.3;

        /*

        geometry_msg::Vector3 linear;
        linear x = velocity;
        velocity_pub.publish(velocity);

        */

        return mbf_msgs::GetPathResult::SUCCESS;
    }

    bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        return false;
    }

    bool  MeshController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        return false;
    }

    bool  MeshController::cancel()
    {
        return false;
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
     * Checks the difficulty of the next vertex to set the speed accordingly
     * @param name
     * @param tf_ptr
     * @param mesh_map_ptr
     * @return indentifier how speed shall be handled
     */
     /*
     int MeshController::lookAhead(current_vertex, next_vertex)
    {
         // depends what type the next_vertex will be:
         // - If its numbers an easy difference can be looked at (here for purple to red increasing numbers)
         //     a) negative => reduce speed
         //     b) positive => increase speed
         //     c) equal => keep speed
         // - If its colors the colors will be assigned numbers first
    }
    */

    /**
     * Turns the robot to align its direction with the path
     * @param name
     * @param tf_ptr
     * @param mesh_map_ptr
     * @return true if robot is aligned; false otherwise
     */
     /*
     bool MeshController::align(robot_angle, path_angle)
    {

         if (robot_angle < path_angle + 0.01 && robot_angle > path_angle - 0.01){
             // robot is aligned to path
             return true
         }
         else if (robot_angle >= path_angle + 0.01){
             // turn robot by 1rad / sec
             return false
         }
         else if (robot_angle <= path_angle - 0.01){
             // turn robot by 1rad / sec
             return false
         }
    }
      */

    bool  MeshController::initialize(
        const std::string& name,
        const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
        const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
    {
        return false;
    }
}