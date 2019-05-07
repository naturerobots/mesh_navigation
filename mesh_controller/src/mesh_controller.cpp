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





        // ANGULAR movement

        // calculate yaw of plan and robot (values between 0 and 2*PI
        float yaw_plan = toEulerAngle(current_plan[iter]);
        float yaw_robot = toEulerAngle(pose);

        float direction_value = 0.0;
        // calculate direction_value depending on the values of yaw_plan and yaw_robot as they can have values
        // between 0 and 2*PI but are ordered in circle
        if (!align(yaw_robot, yaw_plan, 0.1)){
            if(yaw_robot > yaw_plan) {
                direction_value = abs(yaw_robot - yaw_plan);
            } else {        // (yaw_robot < yaw_plan)
                direction_value = yaw_robot + abs(yaw_plan - (2*PI));
            }
            // define turn direction given calculations
            if (direction_value > PI){
                cmd_vel.twist.angular.z = -0.5;        // turn right
                ROS_INFO_STREAM("Turn right");
            } else {
                //cmd_vel.twist.angular.z = 0.25;     // turn left
                ROS_INFO_STREAM("Turn left");
            }
        } else {
            cmd_vel.twist.angular.z = 0.0;
        }


        // LINEAR movement
        // TODO look ahead to determine velocity depending on future vertex cost(s)
        // TODO PID controller
        float new_velocity = 0.2;
        cmd_vel.twist.linear.x = new_velocity;




        // UPDATE ITERATOR to where on the plan the robot is / should be currently
        updatePlanPos(pose, new_velocity);


        return mbf_msgs::GetPathResult::SUCCESS;
    }

    bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        float dist = float(dist_tolerance);
        float angle = float(angle_tolerance);
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
        // TODO figure out how to simplify this (as clion suggests)
        // TODO how to stop velocity without cmd_vel reference
        if (current_x <= goal_x + dist && current_x >= goal_x - dist ||
            current_y <= goal_y + dist && current_y >= goal_y - dist ||
            current_z <= goal_z + dist && current_z >= goal_z - dist){
            // set velocity to zero
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

    float MeshController::toEulerAngle(const geometry_msgs::PoseStamped& pose){
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        float roll;
        float pitch;
        // pose to quaternion values
        float qx = pose.pose.orientation.x;
        float qy = pose.pose.orientation.y;
        float qz = pose.pose.orientation.z;
        float qw = pose.pose.orientation.w;
        // roll (x-axis rotation)
        float sinr_cosp = +2.0 * (qw * qx + qy * qz);
        float cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = +2.0 * (qw * qy - qz * qx);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // yaw (z-axis rotation)
        float siny_cosp = +2.0 * (qw * qz + qx * qy);
        float cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
        return atan2(siny_cosp, cosy_cosp);
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

    void MeshController::updatePlanPos(const geometry_msgs::PoseStamped& pose, float velocity){
        // the faster the robot, the further the look ahead
        int v = static_cast<int>(velocity * 1000.0);
        // smallest[0] is iteration, smallest[1] is euclidean distance
        float smallest[2] = {static_cast<float>(iter), -1};
        for(int j = 0; j < v; j++){
            if(j > current_plan.size()){
                break;
            }
            float dist = euclideanDistance(pose, current_plan[(j + iter)]);
            if (dist < smallest[1] || smallest[1] == -1.0) {
                smallest[0] = j;
                smallest[1] = dist;
            }
        }
        iter += static_cast<int>(smallest[0]);
        current_position = current_plan[iter];
    }
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
        float pos_goal_tolerance = goal_angle + angle_tolerance;
        float neg_goal_tolerance = goal_angle - angle_tolerance;
        if(pos_goal_tolerance >= 2*PI){
            // in case the addition of angle_tolerance overshoots 2*PI - as value range is 0 to 2PI
            pos_goal_tolerance = pos_goal_tolerance - 2*PI;
        }
        if (neg_goal_tolerance <= 0){
            // in chase the subtraction of angle_tolerance is under 0 - as value range is 0 to 2PI
            neg_goal_tolerance = neg_goal_tolerance + 2*PI;
        }

        if (current_angle <= pos_goal_tolerance && current_angle >= neg_goal_tolerance){
            return true;
        } else {
            return false;
        }
    }

    float MeshController::pidControl(const geometry_msgs::PoseStamped& setpoint, const geometry_msgs::PoseStamped& pv){
        // TODO do I have to calculate the supposed setpoint given velocity/angular moment etc or take supposed position on plan?
        // setpoint is desired position, pv is actual position

        // https://gist.github.com/bradley219/5373998
        // calculate error
        float error = euclideanDistance(setpoint, pv);

        // proportional part
        float proportional = prop_gain * error;

        // integral part
        int_error += (error * int_time);
        float integral = int_gain * int_error;

        // derivative part
        float derivative = deriv_gain * ((error - prev_error) / int_time);

        float output = proportional + integral + derivative;

        // TODO check if max and min output useful
        //if( output > _max )
        //    output = _max;
        //else if( output < _min )
        //    output = _min;

        prev_error = error;
        return output;
    }

    bool  MeshController::initialize(
            const std::string& name,
            const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
            const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr){

        // iterator to go through plan vector
        iter = 0;

        int_error = 0.0;
        // int_time has to be higher than 0 -> else division by zero
        int_time = 0.1;
        prev_error = 0.0;
        return true;
    }
}
