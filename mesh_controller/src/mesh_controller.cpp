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

//TODO sort out unnecessary headers

#include <pluginlib/class_list_macros.h>
#include <mesh_controller/mesh_controller.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <lvr2/util/Meap.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <tf/transform_listener.h>


#include <fstream>
#include <iostream>

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
        ){

        if(!current_plan.empty()){
            goal = current_plan.back();
        }


        std::vector<float> values;

        // TODO make usable for directionAtPosition
        mesh_map::Vector plan_vec = poseToDirectionVector(current_position);
        if(useMeshGradient){
            // use supposed orientation from mesh gradient
            if(current_face){
                plan_vec = map_ptr->directionAtPosition(current_face.unwrap(), plan_vec);

            }
        }

        switch(control_type){
            case 1: values = naiveControl(pose, velocity, plan_vec);
            // TODO pid control
            case 2: values = pidControl(pose, pose, velocity);

        }

        // set velocities
        cmd_vel.twist.angular.z = values[0];
        cmd_vel.twist.linear.x = values[1];


        // RECORDING
        if (record) {
            recordData(pose);
        }

        if(!useMeshGradient){
            // UPDATE ITERATOR to where on the plan the robot is / should be currently
            updatePlanPos(pose, values[1]);
        }




        return mbf_msgs::GetPathResult::SUCCESS;
    }

    bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        float dist_tol = dist_tolerance;
        float angle_tol = angle_tolerance;
        float current_angle = angle;

        float current_distance = euclideanDistance(current_position, goal);

        // test if robot is within tolerable distance to goal
        if (current_distance <= dist_tol && current_angle <= angle_tol){
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

    float MeshController::startVelocityFactor(){
        int size = current_plan.size();
        float percentage = (float)iter*100/size;

        if (percentage <= fading){
            return percentage/fading;
        } else {
            return 1.0;
        }
    }

    float MeshController::endVelocityFactor(){
        int size = current_plan.size();
        float percentage = (float)iter*100/size;

        if (percentage >= (100-fading)){
            percentage = 100 - percentage;
            return 1 - (percentage / fading);
        } else {
            return 1.0;
        }

    }

    mesh_map::Vector MeshController::poseToDirectionVector(const geometry_msgs::PoseStamped &pose){
        // define tf Pose for later assignment
        tf::Stamped<tf::Pose> tfPose;
        // transform pose to tf:Pose
        poseStampedMsgToTF(pose, tfPose);

        // get x as tf:Vector of transformed pose
        tf::Vector3 v = tfPose.getBasis()[0];
        // transform tf:Vector in mesh_map Vector and return
        return mesh_map::Vector(v.x(), v.y(), v.z());
    }

    float MeshController::angleBetweenVectors(mesh_map::Vector pos, mesh_map::Vector plan){
        // calculate point product between 2 vectors to get the (smaller) angle between them
        float dot_prod = pos[0]*plan[0] + pos[1]*plan[1] + pos[2]*plan[2];
        float length_pos = sqrt(pow(pos[0], 2) + pow(pos[1], 2) + pow(pos[2], 2));
        float length_plan = sqrt(pow(plan[0], 2) + pow(plan[1], 2) + pow(plan[2], 2));

        return acos(dot_prod / (length_pos * length_plan));
    }

    float MeshController::tanValue(float max_hight, float max_width, float value){
        if (value >= max_width/2){
            return max_hight;
        } else if (value <= -max_width/2) {
            return -max_hight;
        }
        // to widen or narrow the curve
        float width = PI/(max_width);
        float result = max_hight * tan(width * value);
        return result;
    }

    float MeshController::gaussValue(float max_hight, float max_width, float value){
        // in case value lays outside width, the function goes to zero
        if(value > max_width){
            return 0;
        }
        // calculates y value of gaussian distribution for difference, given mean = (center = ) 0 and std. dev = 1
        // it will always have a width of [-3,3] (before being zero) and hight of 0.4
        // transformed value 6 as normal width
        float tr_value = 6.0 * value / max_width;
        float y_value = 1/sqrtf(2*PI) * pow(E, (-0.5 * pow(tr_value, 2)));
        // transform y_value in regards to max_hight
        float result = max_hight * y_value / 0.4;
        return result;
    }

    float MeshController::direction(const mesh_map::Vector& current, const mesh_map::Vector& supposed){
        // cross product
        const mesh_map::Vector& cross_product = {current.y*supposed.z - current.z*supposed.y, current.z*supposed.x - current.x*supposed.z,
                                                 current.x*supposed.y - current.y*supposed.x};
        const mesh_map::Vector& up = {0, 0, 1};
        // dot product of result with up vector (0,0,1)
        float dot_product = cross_product.x*up.x + cross_product.y*up.y + cross_product.z*up.z;

        // TODO check return values for turns
        if(dot_product < 0){
            return -1;
        } else {
            return 1;
        }
    }

    /*
    float MeshController::linearValue(float ref_x_value, float ref_y_value, float y_axis, float value){
        float slope;
        if(ref_x_value == 0){
            __throw_bad_function_call();
        } else {
            slope = (y_axis-ref_y_value)/(0.0-ref_x_value);
        }
        return slope*value + y_axis;
    }
     */


    float MeshController::euclideanDistance(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& plan_position){
        // https://en.wikipedia.org/wiki/Euclidean_distance

        float px = pose.pose.position.x;
        float py = pose.pose.position.y;
        float pz = pose.pose.position.z;

        float cx = plan_position.pose.position.x;
        float cy = plan_position.pose.position.y;
        float cz = plan_position.pose.position.z;

        return euclideanDistance({px, py, pz}, {cx, cy, cz});
    }

    float MeshController::euclideanDistance(lvr2::BaseVector<float> current, lvr2::BaseVector<float> planned){
        float power = 2.0;
        float dist = sqrtf((pow((planned.x-current.x),power) + pow((planned.y-current.y),power) + pow((planned.z-current.z),power)));
        return dist;
    }

    void MeshController::updatePlanPos(const geometry_msgs::PoseStamped& pose, float velocity){
        // the faster the robot, the further the look ahead on planned path
        int v = static_cast<int>(velocity * 1000.0 + 1);
        // smallest[0] is iterator, smallest[1] is euclidean distance
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

    std::vector<float> MeshController::lookAhead(const geometry_msgs::PoseStamped& pose, float velocity)
    {
         // select how far to look ahead depending on velocity
         int steps = (int)tanValue(1000.0, 1.0, velocity);
         // iterator value in case drastic speed reduction has to be made
         float accum_cost = 0.0;
         float accum_turn = 0.0;
         bool vital_turn = false;
         bool vital_cost = false;

         if (useMeshGradient){
             // TODO find out what excactly return vector gives (direction or position)
             lvr2::BaseVector<float> position_ahead = poseToDirectionVector(pose);
             ahead_face = current_face;
             for (int j = 0; j < steps/3; j++){
                 if(euclideanDistance(position_ahead, {goal.pose.position.x, goal.pose.position.y, goal.pose.position.z}) < 0.04){
                     steps = j;
                     break;
                 }
                 position_ahead = step_update(position_ahead, ahead_face.unwrap());
                 float new_cost = cost(position_ahead);
                 if (isinf(new_cost)) {
                     vital_cost = true;
                     break;
                 }

                 float future_turn = angleBetweenVectors(poseToDirectionVector(pose), position_ahead);
                 if (future_turn > PI / 2) {
                     vital_turn = true;
                     break;
                 }

                 accum_cost += new_cost;
                 accum_turn += future_turn;
             }
         } else {
             // adds up cost of all steps ahead
             for (int i = 0; i < steps; i++) {
                 // in case look ahead extends planned path
                 if (iter + i <= current_plan.size()) {
                     steps = i;
                     break;
                 }
                 mesh_map::Vector pose_vec = poseToDirectionVector(current_plan[iter + i]);
                 float new_cost = cost(pose_vec);
                 if (isinf(new_cost)) {
                     vital_cost = true;
                     break;
                 }

                 float future_turn = angleBetweenVectors(poseToDirectionVector(pose), poseToDirectionVector(current_plan[iter + i]));
                 if (future_turn > PI / 2) {
                     vital_turn = true;
                     break;
                 }

                 accum_cost += new_cost;
                 accum_turn += future_turn;
             }
         }


         // average costs
         float av_cost = accum_cost / steps;
         mesh_map::Vector current_vec = poseToDirectionVector(pose);
         float current_cost = cost(current_vec);
         float cost_difference = current_cost - av_cost;

         // average turn CAUTION turn directions can equalize each other
         float av_turn = accum_turn / steps;
         float turn_difference = angle - av_turn;
         float turn_result = tanValue(1.0, PI, turn_difference);

         if (vital_turn || vital_cost) {
             // return turn velocity and reduced speed
             return{turn_result, 0.01};
         }
         //TODO find out what the value range of vector costs is
         return {turn_result, abs(tanValue(1.0, 2.0 * av_cost, cost_difference))};
    }

    float MeshController::cost(mesh_map::Vector& pose_vec){

        if(!haveStartFace){
            current_face = map_ptr->getContainingFaceHandle(pose_vec);
            haveStartFace = true;
        }

        float ret_cost = map_ptr->costAtPosition(current_face.unwrap(), pose_vec);
        if(ret_cost == -1)
        {
            lvr2::OptionalFaceHandle new_face = searchNeighbourFaces(pose_vec, current_face.unwrap());
            if(new_face){
                current_face = new_face;
                ret_cost = map_ptr->costAtPosition(current_face.unwrap(), pose_vec);
            } else {
                return -1;
            }
        }
        return ret_cost;
    }

    lvr2::OptionalFaceHandle MeshController::searchNeighbourFaces(const mesh_map::Vector& pose_vec, const lvr2::FaceHandle face){

        std::list<lvr2::FaceHandle> possible_faces;
        std::vector<lvr2::FaceHandle> neighbour_faces;
        map_ptr->mesh_ptr->getNeighboursOfFace(face, neighbour_faces);
        possible_faces.insert(possible_faces.end(), neighbour_faces.begin(), neighbour_faces.end());
        std::list<lvr2::FaceHandle>::iterator current = possible_faces.begin();

        int cnt = 0;
        int max = 40; // TODO to config


        // as long as end of list is not reached or max steps are not overstepped
        while(possible_faces.end() != current && max != cnt++) {
            lvr2::FaceHandle work_face = *current;

            float u,v;
            // check if robot position is in the current face
            if (map_ptr->barycentricCoords(pose_vec, work_face, u, v)) {
                return work_face;
            } else {
                // add neighbour of neighbour, if we overstep a small face or the peak of it
                std::vector<lvr2::FaceHandle> nn_faces;
                half_edge_mesh.getNeighboursOfFace(work_face, nn_faces);
                possible_faces.insert(possible_faces.end(), nn_faces.begin(), nn_faces.end());
            }
        }
        // in case no face / vertex that contains robot pose is found
        return lvr2::OptionalFaceHandle();
    }

    std::vector<float> MeshController::naiveControl(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,const mesh_map::Vector plan_vec){
        mesh_map::Vector pose_vec = poseToDirectionVector(pose);

        // calculate angle between orientation vectors
        // angle will never be negative and smaller or equal to pi
        angle = angleBetweenVectors(pose_vec, plan_vec);

        // to determine in which direction to turn (neg for left, pos for right)
        float leftRight = direction(pose_vec, plan_vec);


        float turn_angle_diff = leftRight * tanValue(0.5, PI, angle);



        // LINEAR movement

        // basic linear velocity depending on angle difference between robot pose and plan
        float vel_given_angle = gaussValue(maximum_lin_velocity, PI, angle);

        // look ahead
        std::vector<float> ahead_factor = lookAhead(pose, velocity.twist.linear.x);

        // dynamic obstacle avoidance

        // slow start
        float start = startVelocityFactor();
        // slower towards end
        float end = endVelocityFactor();


        float new_turn = turn_angle_diff * ahead_factor[0];
        float new_vel = start * end * ahead_factor[1] * vel_given_angle;

        return {new_turn, new_vel};

    }

    std::vector<float> MeshController::pidControl(const geometry_msgs::PoseStamped& setpoint, const geometry_msgs::PoseStamped& pv, const geometry_msgs::TwistStamped& velocity){
        float linear_vel = pidControlDistance(setpoint, pv);

        const mesh_map::Vector& angular_sp = poseToDirectionVector(setpoint);
        const mesh_map::Vector& angular_pv = poseToDirectionVector(pv);
        float angular_vel = pidControlDir(angular_sp, angular_pv);

        // to regulate linear velocity depending on angular velocity (higher angular vel => lower linear vel)
        float linear_factor = gaussValue(maximum_lin_velocity, 2*maximum_ang_velocity, angular_vel);

        std::vector<float> ahead = lookAhead(pv, velocity.twist.linear.x);

        float new_angular = angular_vel * ahead[0];
        float new_linear = linear_vel * linear_factor * ahead[1];

        return {new_angular, new_linear};
    }

    float MeshController::pidControlDistance(const geometry_msgs::PoseStamped& setpoint, const geometry_msgs::PoseStamped& pv){
        // setpoint is desired position, pv is actual position
        // https://gist.github.com/bradley219/5373998

        float error = euclideanDistance(setpoint, pv);

        // proportional part
        float proportional = prop_dis_gain * error;

        // integral part
        int_error += (error * int_time);
        float integral = int_dis_gain * int_error;

        // derivative part
        float derivative = deriv_dis_gain * ((error - prev_distance_error) / int_time);

        float linear = proportional + integral + derivative;

        // TODO check if max and min output useful
        //if( output > _max )
        //    output = _max;
        //else if( output < _min )
        //    output = _min;

        prev_distance_error = error;
        return linear;
    }

    float MeshController::pidControlDir(const mesh_map::Vector& setpoint, const mesh_map::Vector& pv){
        // setpoint is desired position, pv is actual position
        // https://gist.github.com/bradley219/5373998

        float dir_error = angleBetweenVectors(setpoint, pv);

        // proportional part
        float proportional = prop_dir_gain * dir_error;

        // integral part
        int_error += (dir_error * int_time);
        float integral = int_dir_gain * int_error;

        // derivative part
        float derivative = deriv_dir_gain * ((dir_error - prev_dir_error) / int_time);

        float angular = proportional + integral + derivative;

        // TODO check if max and min output useful
        //if( output > _max )
        //    output = _max;
        //else if( output < _min )
        //    output = _min;

        prev_dir_error = dir_error;

        // to determine in which direction to turn (neg for left, pos for right)
        float leftRight = direction(pv, setpoint);

        return angular*leftRight;
    }

    void MeshController::recordData(const geometry_msgs::PoseStamped& robot_pose){
        string output = std::to_string(robot_pose.pose.position.x) + "\t" + std::to_string(robot_pose.pose.position.y)
                + "\t" + std::to_string(robot_pose.pose.position.z) + "\t" + std::to_string(current_position.pose.position.x)
                + "\t" + std::to_string(current_position.pose.position.y) + "\t" + std::to_string(current_position.pose.position.z) + "\n";

        string info_msg = string("robot_pose x") + "\t" + "robot_pose y" + "\t" + "robot_pose z" + "\t" + "plan_pose x"
                + "\t" + "plan_pose y" + "\t" + "plan_pose z" + "\n";

        const char* filename = "Paths.txt";
        fstream appendFileToWorkWith;

        appendFileToWorkWith.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);


        // If file does not exist, Create new file
        if (!appendFileToWorkWith )
        {
            appendFileToWorkWith.open(filename,  fstream::in | fstream::out | fstream::trunc);
            appendFileToWorkWith <<info_msg;
            appendFileToWorkWith <<output;
            appendFileToWorkWith.close();

        }
        else
        {    // use existing file
            appendFileToWorkWith <<output;
            appendFileToWorkWith.close();

        }
    }

    lvr2::BaseVector<float> MeshController::step_update(mesh_map::Vector& vec, lvr2::FaceHandle face){
        // clear vector field map
        vector_map.clear();

        const auto& face_normals = map_ptr->faceNormals();

        bool foundConnectedFace = false;
        std::list<lvr2::FaceHandle> possible_faces;
        std::vector<lvr2::FaceHandle> neighbour_faces;
        map_ptr->mesh_ptr->getNeighboursOfFace(face, neighbour_faces);
        possible_faces.insert(possible_faces.end(), neighbour_faces.begin(), neighbour_faces.end());
        std::list<lvr2::FaceHandle>::iterator current = possible_faces.begin();
        mesh_map::Vector dir;
        float step_width = 0.03;

        // Set start distance to zero
        // add start vertex to priority queue
        for(auto vH : map_ptr->mesh_ptr->getVerticesOfFace(face))
        {
            const mesh_map::Vector diff = vec - map_ptr->mesh_ptr->getVertexPosition(vH);
            vector_map.insert(vH, diff);
        }



        int cnt = 0;
        int max = 40; // TODO to config

        while(possible_faces.end() != current && max != cnt++)
        {
            lvr2::FaceHandle fH = *current;
            auto vertices = map_ptr->mesh_ptr->getVertexPositionsOfFace(fH);
            auto face_vertices = map_ptr->mesh_ptr->getVerticesOfFace(fH);
            float u, v;

            // Projection onto the triangle plane
            mesh_map::Vector tmp_vec = mesh_map::projectVectorOntoPlane(vec, vertices[0], face_normals[fH]);

            // Check if the projected point lies in the current testing face
            if(vector_map.containsKey(face_vertices[0]) && vector_map.containsKey(face_vertices[1]) && vector_map.containsKey(face_vertices[2])
               && mesh_map::barycentricCoords(tmp_vec, vertices[0], vertices[1], vertices[2], u, v))
            {
                foundConnectedFace = true;
                // update ahead_face as face of the new vector
                ahead_face = fH;
                vec = tmp_vec;
                float w = 1 - u - v;
                dir = ( vector_map[face_vertices[0]]*u + vector_map[face_vertices[1]]*v + vector_map[face_vertices[2]]*w ).normalized() * step_width ;
                break;
            }
            else
            {
                // add neighbour of neighbour, if we overstep a small face or the peak of it
                std::vector<lvr2::FaceHandle> nn_faces;
                map_ptr->mesh_ptr->getNeighboursOfFace(fH, nn_faces);
                possible_faces.insert(possible_faces.end(), nn_faces.begin(), nn_faces.end());
            }
            current++;
        }

        if(!foundConnectedFace){
            return lvr2::BaseVector<float>();
        }

        return vec + dir;
    }

    /*
    void MeshController::obstacleAvoidance(pose, mesh_map, path){
        // make new measurement + make mesh layer of it
        // compare new measurement to old one - find new lethal faces
        // check if new lethal faces are on path
            // if on path: reduce velocity + make new plan

    }
    */

    bool  MeshController::initialize(
            const std::string& name,
            const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
            const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr){

        // all for mesh plan
        map_ptr = mesh_map_ptr;


        // iterator to go through plan vector
        iter = 0;

        int_error = 0.0;
        // int_time has to be higher than 0 -> else division by zero
        int_time = 0.1;
        haveStartFace = false;
        fading = 5;

        // for recording - true = record, false = no record
        record = false;

        prop_dis_gain = 1.0;
        int_dis_gain = 1.0;
        deriv_dis_gain = 1.0;
        prop_dir_gain = 1.0;
        int_dir_gain = 1.0;
        deriv_dir_gain = 1.0;
        /*
        // pid controller tuning parameters
        float k_u = ; float t_u = ;
        float tau = ; g_p = ; t_d = ;
        switch(tuning_type){
            case 1: prop_gain = 0.6*k_u; int_gain =0.5*t_u ; deriv_gain = 0.125*t_u; //ZN https://dewesoft.pro/online/course/pid-control/page/pid-tuning-methods
            case 2: prop_gain = k_u/2.2; int_gain = 2.2*t_u; deriv_gain = t_u/6.3; // T-L  http://pages.mtu.edu/~tbco/cm416/zn.html
            case 3: prop_gain = (1.35/g_t)*(tau/t_d + 0.185); int_gain = 2.5 * t_d * ((tau+0.185*t_d)/tau + 0.611*t_d); deriv_gain = 0.37 * t_d * tau / (tau + 0.185*t_d); // C-C https://dewesoft.pro/online/course/pid-control/page/pid-tuning-methods
            case 4: prop_gain = ; int_gain = ; deriv_gain = ;
                prop_gain = ; int_gain = ; deriv_gain =

        }
        */

        return true;

    }
} /* namespace mesh_controller */
