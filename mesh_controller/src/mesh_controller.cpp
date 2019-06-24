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
#include <std_msgs/Float32.h>



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
        // check if a path has been calculated and given
        if(current_plan.empty()){
            return mbf_msgs::GetPathResult::EMPTY_PATH;
        } else if (!goalSet){
            goal = current_plan.back();
            goalSet= true;
        }

        // set current face
        mesh_map::Vector pos_vec = poseToPositionVector(pose);


        setCurrentFace(pos_vec);

        // update to which position of the plan the robot is closest
        if (!config.useMeshGradient) {
            // update to which position of the plan the robot is closest
            updatePlanPos(pose, set_linear_velocity);
            // check if the robot is too far off the plan
            if (offPlan(pose)) {
                // TODO see if NOT_INITIALIZED = 112 can be used
                // TODO check if mesh gradient should be used then
                return mbf_msgs::GetPathResult::FAILURE;
            }
        }

        // variable that contains the planned / supposed orientation of the robot at the current position
        mesh_map::Vector supposed_heading;
        if (!config.useMeshGradient) {
            // use supposed orientation from calculated path
            supposed_heading = poseToDirectionVector(plan_position);
        } else {
            auto vertices = map_ptr->mesh_ptr->getVerticesOfFace(current_face.unwrap());
            if (vector_map.containsKey(vertices[0]) && vector_map.containsKey(vertices[1]) && vector_map.containsKey(vertices[2])){
                ROS_INFO("vertex exists in map");
            } else {
                ROS_INFO("vertex NOT in map");
            }
            float u, v, w;
            if(map_ptr->barycentricCoords(pos_vec, current_face.unwrap(), u, v))
            {
                w = 1 - u - v;
                supposed_heading = vector_map[vertices[0]] * u + vector_map[vertices[1]] * v + vector_map[vertices[2]] * w;
            }
            /*
            try{
                supposed_heading = map_ptr->directionAtPosition(current_face.unwrap(), pos_vec);
            } catch(...) {
                auto vertices = map_ptr->mesh_ptr->getVerticesOfFace(current_face.unwrap());
                if(vector_map.containsKey(vertices[0])){
                    ROS_ERROR("vertex found in vector map");
                } else {
                    ROS_ERROR("vertex NOT found in vector map");
                }
            }*/
        }

        // variable to store the new angular and linear velocities
        std::vector<float> values(2);

        // determine values via naive controller
        values = naiveControl(pose, supposed_heading);


        if (values[1] == std::numeric_limits<float>::max()){
            return mbf_msgs::GetPathResult::FAILURE;
        }
        // set velocities
        cmd_vel.twist.angular.z = values[0];
        cmd_vel.twist.linear.x = values[1];

        return mbf_msgs::GetPathResult::SUCCESS;
    }

    bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        float dist;
        // calculates the distance that is currently between the plan position and the goal
        if(!config.useMeshGradient){
            tf::Pose plan_pose, goal_pose;
            tf::poseMsgToTF(plan_position.pose, plan_pose);
            tf::poseMsgToTF(goal.pose, goal_pose);

            dist = plan_pose.getOrigin().distance(goal_pose.getOrigin());
        } else {
            dist = goal_dist_mesh;
        }


        // test if robot is within tolerable distance to goal and if the heading has a tolerable distance to goal heading
        if (dist <= (float)dist_tolerance && angle <= (float)angle_tolerance){
            return true;
        } else {
            return false;
        }
    }

    bool  MeshController::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        // checks if the given vector contains the plan
        if (!plan.empty()) {
            // assign given plan to current_plan variable to make it usable for navigation
            current_plan = plan;
            // erase first plan pose as it (usually) contains a unuseful direction
            current_plan.erase(current_plan.begin());
            // set goal according to plan
            goal = current_plan.back();
            // set initial distance as infinite as indicator for later initializations
            initial_dist = std::numeric_limits<float>::max();
            // set the variable vector_map with the current vector map
            vector_map = map_ptr->getVectorMap();
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

    float MeshController::fadingFactor(const geometry_msgs::PoseStamped& pose){
        // calculating the fading factor when using the path for naviagation
        if (!config.useMeshGradient) {
            float dist = 0.0;
            // calculate the distance between neighbouring plan positions to get the whole path length
            if (initial_dist == std::numeric_limits<float>::max()) {
                // transform the first pose of plan to a tf position vector
                tf::Vector3 tf_start_vec;
                geometry_msgs::PoseStamped start_pose = current_plan.at(0);
                mesh_map::Vector start_vec = poseToPositionVector(start_pose);
                tf_start_vec = {start_vec.x, start_vec.y, start_vec.z};

                tf::Vector3 tf_next_vec;
                float update_dist = 0.0;

                for (int i = 1; i < current_plan.size(); i++) {
                    // get the next pose of the plan and transform it to a tf position vector
                    geometry_msgs::PoseStamped next_pose = current_plan.at(i);
                    mesh_map::Vector next_vec = poseToPositionVector(next_pose);
                    tf_next_vec = {next_vec.x, next_vec.y, next_vec.z};
                    // calculate the distance between the two vectors
                    update_dist = update_dist + tf_start_vec.distance(tf_next_vec);

                    // overwrite start position with next position to calculate the difference
                    // between the next and next-next position in next step
                    tf_start_vec = tf_next_vec;
                }
                initial_dist = update_dist;
            }

            // transform the first pose of plan to a tf position vector
            tf::Vector3 tf_start_vec;
            geometry_msgs::PoseStamped start_pose = current_plan.at(0);
            mesh_map::Vector start_vec = poseToPositionVector(start_pose);
            tf_start_vec = {start_vec.x, start_vec.y, start_vec.z};


            tf::Vector3 tf_next_vec;

            // add up distance of travelled path
            for (int i = 1; i <= plan_iter && i < current_plan.size(); i++) {
                geometry_msgs::PoseStamped next_pose = current_plan.at(i);
                mesh_map::Vector next_vec = poseToPositionVector(next_pose);
                tf_next_vec = {next_vec.x, next_vec.y, next_vec.z};

                dist += tf_start_vec.distance(tf_next_vec);
                // overwrites start position with next position to calculate the difference
                // between the next and next-next position in next step
                tf_start_vec = tf_next_vec;

            }
            // compare the now travelled distance with the path length
            // in case the travelled distance is close to start position
            if (dist < config.fading) {
                if (dist == 0.0) {
                    // return small factor in case of initial position to enable movement
                    // note: if max_velocity is zero, this factor will not matter
                    last_fading = config.max_lin_velocity / 10;
                    return last_fading;
                }
                // returns a factor slowly increasing to 1 while getting closer to the
                // distance from which the full velocity is driven
                return dist / config.fading;
            }
                // in case the travelled distance is close to goal position
            else if ((initial_dist - dist) < config.fading) {
                // returns a factor slowly decreasing to 0 the end of the full velocity towards the goal position
                last_fading = (initial_dist - dist) / config.fading;
                return last_fading;
            }
            // for the part of the path when the velocity does not have to be influenced by a changing factor
            last_fading = 1.0;
            return last_fading;
        }
        // for mesh gradient use
        else {
            // When the mesh gradient is used only the beeline can be used to ESTIMATE the distance between
            // the start position and the goal as no knowledge about the path that will be travelled is available.
            // If this should be more accurate than a a path would have to be calculated which is not the intention
            // when using the mesh gradient for navigation.
            if (initial_dist == std::numeric_limits<float>::max()){
                // the position from which the distance is calculated is the first pose of the robot
                geometry_msgs::PoseStamped start_pose = current_plan.at(0);
                mesh_map::Vector start_vec = poseToPositionVector(start_pose);
                mesh_map::Vector goal_vec = poseToPositionVector(goal);

                initial_dist = euclideanDistance(start_vec, goal_vec);
            }

            // calculate the distance between the current robot position and the start position as well as the
            // distance between the current robot position and the goal position
            geometry_msgs::PoseStamped start_pose = current_plan.at(0);
            mesh_map::Vector start_vec = poseToPositionVector(start_pose);
            mesh_map::Vector goal_vec = poseToPositionVector(goal);
            mesh_map::Vector current_vec = poseToPositionVector(pose);

            float dist_start = euclideanDistance(start_vec, current_vec);
            float dist_goal = euclideanDistance(current_vec, goal_vec);
            // set the global variable goal_dist_mesh to be able to check if the goal is reached
            goal_dist_mesh = dist_goal;

            // compare the now travelled distance with the path length
            // in case the travelled distance is close to start position
            if (dist_start < config.fading) {
                if (dist_start == 0.0) {
                    // return small factor in case of initial position to enable movement
                    // note: if max_velocity is zero, this factor will not matter
                    last_fading = config.max_lin_velocity / 10;
                    return last_fading;
                }
                // returns a factor slowly increasing to 1 while getting closer to the
                // distance from which the full velocity is driven
                return dist_start / config.fading;
            }
            // in case the travelled distance is close to goal position
            else if (dist_goal < config.fading) {
                // returns a factor slowly decreasing to 0 the end of the full velocity towards the goal position
                last_fading = 1 - (dist_goal / config.fading);
                return last_fading;
            }
            // for the part of the path when the velocity does not have to be influenced by a changing factor
            last_fading = 1.0;
            return last_fading;
        }
    }

    mesh_map::Vector MeshController::poseToDirectionVector(const geometry_msgs::PoseStamped &pose){
        // define tf Pose for later assignment
        tf::Stamped<tf::Pose> tfPose;
        // transform pose to tf:Pose
        poseStampedMsgToTF(pose, tfPose);

        // get x as tf:Vector of transformed pose
        tf::Vector3 v = tfPose.getBasis()*tf::Vector3(1, 0, 0);
        // transform tf:Vector in mesh_map Vector and return
        return mesh_map::Vector(v.x(), v.y(), v.z());
    }

    mesh_map::Vector MeshController::poseToPositionVector(const geometry_msgs::PoseStamped &pose){
        return {(float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z};
    }

    float MeshController::angleBetweenVectors(mesh_map::Vector robot_heading, mesh_map::Vector planned_heading){
        tf::Vector3 tf_robot(robot_heading.x, robot_heading.y, robot_heading.z);
        tf::Vector3 tf_planned(planned_heading.x, planned_heading.y, planned_heading.z);
        return tf_robot.angle(tf_planned);
    }

    float MeshController::linValue(float max_hight, float x_axis, float max_width, float value){
        // checking higher and lower boarders
        if (value > max_width/2){
            return max_hight;
        } else if (value < -max_width/2){
            return -max_hight;
        }
        // calculating the incline of the linear function
        float incline = max_hight / (max_width/2);
        return abs(incline * (value + x_axis));
    }

    float MeshController::gaussValue(float max_hight, float max_width, float value){
        // in case value lays outside width, the function goes to zero
        if(value > max_width/2){
            return 0.0;
        }

        //  calculating the standard deviation given the max_width
        // based on the fact that 99.7% of area lies between mü-3*sigma and mü+3*sigma
        float std_dev = pow(-max_width/6, 2);

        // calculating y value of given normal distribution
        // stretched to max_hight and desired width
        float y_value = max_hight * 1/(sqrtf(2*M_PI*std_dev))* pow(E, (-pow(value, 2) * pow((2*std_dev), 2)));

        return y_value;
    }

    float MeshController::direction(mesh_map::Vector& robot_heading, mesh_map::Vector& planned_heading){

        tf::Vector3 tf_robot(robot_heading.x, robot_heading.y, robot_heading.z);
        tf::Vector3 tf_planned(planned_heading.x, planned_heading.y, planned_heading.z);

        https://www.gamedev.net/forums/topic/508445-left-or-right-direction/
        tf::Vector3 tf_cross_prod = tf_robot.cross(tf_planned);

        tf::Vector3 tf_up = {0,0,-1};

        // use normal vector of face for dot product as "up" vector
        // => positive result = left, negative result = right,
        float tf_dot_prod = tf_cross_prod.dot(tf_up);

        if (tf_dot_prod < 0.0) {
            // left turn
            return 1.0;
        } else {
            // right turn
            return -1.0;
        }
    }

    bool MeshController::offPlan(const geometry_msgs::PoseStamped& robot_pose){
        // transform the first pose of plan to a tf position vector
        mesh_map::Vector robot_vec = poseToPositionVector(robot_pose);
        mesh_map::Vector plan_vec = poseToPositionVector(plan_position);

        if(euclideanDistance(robot_vec, plan_vec) > config.off_plan){
            return true;
        }
        return false;
    }

    float MeshController::euclideanDistance(const geometry_msgs::PoseStamped& pose){
        mesh_map::Vector pose_vec = poseToPositionVector(pose);
        tf::Vector3 tf_pose_vec = {pose_vec.x, pose_vec.y, pose_vec.z};

        mesh_map::Vector goal_vec = poseToPositionVector(goal);
        tf::Vector3 tf_goal_vec = {goal_vec.x, goal_vec.y, goal_vec.z};

        return tf_pose_vec.distance(tf_goal_vec);
    }

    float MeshController::euclideanDistance(lvr2::BaseVector<float>& current, lvr2::BaseVector<float>& planned){
        tf::Vector3 tf_current_vec = {current.x, current.y, current.z};
        tf::Vector3 tf_planned_vec = {planned.x, planned.y, planned.z};

        return tf_current_vec.distance(tf_planned_vec);
    }

    void MeshController::updatePlanPos(const geometry_msgs::PoseStamped& pose, float velocity){
        // checking if the function has been called before
        // if not - set the time for the first time
        if(last_call.isZero())
        {
            last_call = ros::Time::now();
            // initialize the iterator for the planned path with zero = start position
            plan_iter = 0;
            return;
        }
        // calculate the time between now and the last call of the function
        ros::Time now = ros::Time::now();
        ros::Duration time_delta = now - last_call;

        // determine the maximum distance that the robot could have driven between the last function call
        // and now based on its velocity
        // reason : the faster the robot, the further the robot might have travelled on the planned path
        double max_dist = velocity * time_delta.toSec();
        float min_dist = std::numeric_limits<float>::max();

        // transform the current robot pose to a position vector
        mesh_map::Vector robot_vec = poseToPositionVector(pose);

        int iter, ret_iter;
        // iterator from which to start search
        iter = plan_iter;

        float dist = 0;
        // look forward on the plan and check if another position on the plan is closer to the robot
        do{
            // get the position on the path
            geometry_msgs::PoseStamped iter_pose = current_plan[iter];
            // transform it to a position vector
            mesh_map::Vector iter_vec = poseToPositionVector(iter_pose);
            // calculate the distance between the robot position and the planned path position
            dist = euclideanDistance(robot_vec, iter_vec);
            // if the distance is smaller than the previously calculated one, save the values
            if(dist < min_dist){
                ret_iter = iter;
                min_dist = dist;
            }
            iter++;
        }
        // repeat this until either the maximum travelled distance is reached or the planned path ends
        while(dist > max_dist && iter < current_plan.size());

        // reset iterator
        iter = plan_iter;

        // look back, as above but going backwards through the plan
        do{
            geometry_msgs::PoseStamped iter_pose = current_plan[iter];
            mesh_map::Vector iter_vec = poseToPositionVector(iter_pose);
            dist = euclideanDistance(robot_vec, iter_vec);
            if(dist < min_dist){
                ret_iter = iter;
                min_dist = dist;
            }
            iter--;
        }
        // do this until either the maximum travelled distance is or the start of the planned path is reached
        while(dist > max_dist && iter >= 0);

        // update the values
        // plan iter becomes the iterator of the position with the smallest distance to the robot
        plan_iter = ret_iter;
        // plan position becomes the position of the plan with the smallest distance to the robot
        plan_position = current_plan[plan_iter];
        last_call = now;
    }

    std::vector<float> MeshController::lookAhead(const geometry_msgs::PoseStamped& pose, float velocity)
    {
        mesh_map::Vector robot_heading = poseToDirectionVector(pose);
        mesh_map::Vector robot_position = poseToPositionVector(pose);

        // initialize time of last look ahead call when function has not been called before
        if(last_lookahead_call.isZero())
        {
            last_lookahead_call = ros::Time::now();
            return {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        }
        // determine the time that has passed since the last look ahead
        ros::Time now = ros::Time::now();
        ros::Duration time_delta = now - last_lookahead_call;

        // calculate the maximum distance the robot could have travelled since the last function call
        // because: the faster the robot, the further the distance that may be travelled and therefore the look ahead
        double max_travelled_dist = velocity * time_delta.toSec();

        // select how far to look ahead depending on the max travelled distance
        int steps;
        if (!config.useMeshGradient){
            // calculates approximately how many elements of the path have to be called and travelled along to
            // reach the maximum distance that could be travelled with the given velocity
            steps = (int)linValue(current_plan.size(), 0.0, 2*initial_dist, max_travelled_dist)-plan_iter;
        } else {
            // calculates how many steps ahead have to be checked when travelling in the current robot direction
            // with the given velocity by dividing it through the step size that is later used
            // therefore the look ahead will be as far as the maximum travelled distance
            steps = (int)(max_travelled_dist/0.03);
        }


        if (steps == 0){
            // no look ahead when there is no linear velocity
            // values have to be dealt with after function call
            return {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        }


        // variable to add how many times the cost could not be calculated and therefore not added up
        int missed_steps = 0;
        // variable to store on which position of the plan a lethal vertex will be first encountered
        int lethal_step = 0;
        // variables to accumulate the cost and direction values of the future positions
        float accum_cost = 0.0;
        float accum_turn = 0.0;
        // face handle to store the face of the position ahead
        lvr2::OptionalFaceHandle future_face;
        // initialize the face ahead with the current face
        if (!config.useMeshGradient){
            future_face = current_face;
        } else {
            mesh_ahead_face = current_face;
        }

        float new_cost;
        mesh_map::Vector ahead_position_vec;
        mesh_map::Vector ahead_direction_vec;
        // adds up cost and angles of all steps ahead
        for (int i = 0; i <= steps; i++) {
            // look ahead when using the planned path for navigation reference
            if (!config.useMeshGradient) {
                // in case look ahead extends planned path
                if ((plan_iter + i) >= current_plan.size()) {
                    steps = i;
                    break;
                }

                // gets the pose of the ahead position
                geometry_msgs::PoseStamped pose_ahead = current_plan[plan_iter + i];
                // converts the pose to a position vector
                ahead_position_vec = poseToPositionVector(pose_ahead);
                // converts the pose into a direction vector
                ahead_direction_vec = poseToDirectionVector(pose_ahead);
                // finds the face which contains the current ahead face
                future_face = setAheadFace(future_face.unwrap(), ahead_position_vec);

                // find cost of the future position
                new_cost = cost(future_face, ahead_position_vec);
            }
            // look ahead for using mesh gradient for navigation reference
            else {
                // if the step size is zero, check one step from current robot position
                if (i == 0){
                    ahead_position_vec = meshAhead(robot_position, mesh_ahead_face.unwrap());
                }
                // otherwise use the previously calculated ahead face and position to calculate the next step
                else {
                    ahead_position_vec = meshAhead(ahead_position_vec, mesh_ahead_face.unwrap());
                }
                // determine the direction and cost at the new positions
                ahead_direction_vec = map_ptr->directionAtPosition(mesh_ahead_face.unwrap(), ahead_position_vec);
                new_cost = cost(mesh_ahead_face, ahead_position_vec);
            }


            if (new_cost == std::numeric_limits<float>::infinity() && lethal_step == 0){
                ROS_INFO_STREAM("lethal vertex "<<i);
                lethal_step = i;
            } else if (new_cost == -1.0){
                ROS_INFO("cost could not be accessed");
                // cost could not be accessed
                // CAUTION could lead to division by zero
                missed_steps += 1;
            } else {
                // get direction difference between current position and next position
                float future_turn = angleBetweenVectors(robot_heading,
                                                        ahead_direction_vec);
                // to determine in which direction to turn in future (neg for left, pos for right)
                float leftRight = direction(robot_heading, ahead_direction_vec);
                // accumulate cost and angle
                accum_cost += new_cost;
                accum_turn += (future_turn*leftRight);
            }
        }


        if ((steps - missed_steps) <= 0 || steps <= 0) {
            return {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        }
        //  take averages of future values
        float av_turn = accum_turn / (steps - missed_steps);
        float av_cost = accum_cost / (steps - missed_steps);

        return {av_turn, av_cost};
    }

    float MeshController::cost(mesh_map::Vector& pose_vec){
        // call function to get cost at current position through corresponding face
        float ret_cost = map_ptr->costAtPosition(current_face.unwrap(), pose_vec);
        return ret_cost;
    }

    float MeshController::cost(lvr2::OptionalFaceHandle face, mesh_map::Vector& position_vec){
        // call function to get cost at position through corresponding face
        float ret_cost = map_ptr->costAtPosition(face.unwrap(), position_vec);
        return ret_cost;
    }

    void MeshController::setCurrentFace(mesh_map::Vector& position_vec){
        // check if current face is already set
        if(!current_face){
            // search through mesh faces to find face containing the position
            current_face = map_ptr->getContainingFaceHandle(position_vec);
            if(!current_face){
                ROS_ERROR("searched through mesh - no current face");
            } else {
                return;
            }
        } else {
            // search through neighbours of the last set face to find face that contains position
            current_face = searchNeighbourFaces(position_vec, current_face.unwrap());
            // if no neighbour is fond that contains position, call the function again
            if(!current_face){
                setCurrentFace(position_vec);
            } else {
                return;
            }
        }
    }

    lvr2::OptionalFaceHandle MeshController::setAheadFace(lvr2::OptionalFaceHandle face, mesh_map::Vector& position_vec){
        lvr2::OptionalFaceHandle next_face;
        if(!face){
            // iterate over all faces to find the face containing the position
            next_face = map_ptr->getContainingFaceHandle(position_vec);
            if(!next_face) {
                ROS_ERROR("searched through mesh - no ahead face");
            } else {
                // returns the face of the position when found
                return next_face;
            }
        } else {
            next_face = searchNeighbourFaces(position_vec, face.unwrap());
            if(!next_face){
                // call the function recursively with the position and an empty face
                // => iteration over all faces to find face
                setAheadFace(next_face, position_vec);
            } else {
                // returns the face of the position when found
                return next_face;
            }
        }
    }

    lvr2::OptionalFaceHandle MeshController::searchNeighbourFaces(const mesh_map::Vector& pose_vec, const lvr2::FaceHandle face){
        std::list<lvr2::FaceHandle> possible_faces;
        possible_faces.push_back(face);
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
                map_ptr->mesh_ptr->getNeighboursOfFace(work_face, nn_faces);
                possible_faces.insert(possible_faces.end(), nn_faces.begin(), nn_faces.end());
            }
        }

        return lvr2::OptionalFaceHandle();
    }

    lvr2::BaseVector<float> MeshController::meshAhead(mesh_map::Vector& vec, lvr2::FaceHandle face){

        const auto& face_normals = map_ptr->faceNormals();

        constexpr float step_width = 0.03;

        // steps to take to project vector onto face
        lvr2::FaceHandle used_face = face;
        auto face_vertices = map_ptr->mesh_ptr->getVerticesOfFace(used_face);
        auto vertices = map_ptr->mesh_ptr->getVertexPositionsOfFace(used_face);
        mesh_map::Vector normal_vec = mesh_map::projectVectorOntoPlane(vec, vertices[0], face_normals[used_face]);

        mesh_map::Vector dir;


        float u, v;
        // check if vector is in current face
        if(mesh_map::barycentricCoords(normal_vec, vertices[0], vertices[1], vertices[2], u, v))
        {
            float w = 1 - u - v;
            dir = ( vector_map[face_vertices[0]]*u + vector_map[face_vertices[1]]*v + vector_map[face_vertices[2]]*w ).normalized() * step_width;
        }
        else
        {
            bool foundConnectedFace = false;
            std::list<lvr2::FaceHandle> possible_faces;
            std::vector<lvr2::FaceHandle> neighbour_faces;
            map_ptr->mesh_ptr->getNeighboursOfFace(face, neighbour_faces);
            possible_faces.insert(possible_faces.end(), neighbour_faces.begin(), neighbour_faces.end());
            std::list<lvr2::FaceHandle>::iterator current = possible_faces.begin();

            int cnt = 0;
            int max = 40; // TODO to config

            while(possible_faces.end() != current && max != cnt++)
            {
                // steps to take to project vector onto neighbour face
                lvr2::FaceHandle fH = *current;
                vertices = map_ptr->mesh_ptr->getVertexPositionsOfFace(fH);
                face_vertices = map_ptr->mesh_ptr->getVerticesOfFace(fH);
                mesh_map::Vector tmp_vec = mesh_map::projectVectorOntoPlane(vec, vertices[0], face_normals[fH]);

                // Check if the projected point lies in the current testing face
                if(vector_map.containsKey(face_vertices[0]) && vector_map.containsKey(face_vertices[1]) && vector_map.containsKey(face_vertices[2])
                   && mesh_map::barycentricCoords(tmp_vec, vertices[0], vertices[1], vertices[2], u, v))
                {
                    foundConnectedFace = true;
                    used_face = fH;
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
                ROS_ERROR_STREAM("Sample path failed! Could not find a connected face in vector direction!");
                return lvr2::BaseVector<float>();
            }
        }

        mesh_ahead_face = used_face;
        return vec + dir;

    }

    std::vector<float> MeshController::naiveControl(const geometry_msgs::PoseStamped& pose, mesh_map::Vector supposed_dir) {
        mesh_map::Vector dir_vec = poseToDirectionVector(pose);
        mesh_map::Vector position_vec = poseToPositionVector(pose);

        // ANGULAR MOVEMENT
        // calculate angle between orientation vectors
        // angle will never be negative and smaller or equal to pi
        angle = angleBetweenVectors(dir_vec, supposed_dir);

        // output: angle publishing
        std_msgs::Float32 angle32;
        angle32.data = angle * 180 / M_PI;
        angle_pub.publish(angle32);

        // to determine in which direction to turn (neg for left, pos for right)
        float leftRight = direction(dir_vec, supposed_dir);

        // calculate a direction velocity depending on the turn direction and difference angle
        float final_ang_vel = leftRight * linValue(config.max_ang_velocity, 0.0, 2 * M_PI, angle);

        // LINEAR movement
        float lin_vel_by_ang = gaussValue(config.max_lin_velocity, 2 * M_PI, angle);
        float final_lin_vel;

        // check the size of the angle. If it is not more than about 35 degrees, integrate position costs to linear velocity
        if (angle < 0.6) {
            float cost_lin_vel = cost(position_vec);
            // in case current vertex is a lethal vertex
            if (cost_lin_vel != std::numeric_limits<float>::max()) {
                // basic linear velocity depending on angle difference between robot pose and plan and the cost at position
                float lin_factor_by_cost = linValue(config.max_lin_velocity/10,  0.0, 2.0, cost_lin_vel);
                lin_vel_by_ang -= lin_factor_by_cost;
                if (lin_vel_by_ang < 0.0) {
                    lin_vel_by_ang = 0.0;
                } else if (lin_vel_by_ang > config.max_lin_velocity) {
                    lin_vel_by_ang = config.max_lin_velocity;
                }
            }
        }

        final_lin_vel = lin_vel_by_ang;

        // ADDITIONAL factors
        // look ahead
        std::vector<float> ahead_values = lookAhead(pose, set_linear_velocity);

        if(ahead_values[1] != std::numeric_limits<float>::max()) {
            // get the direction factor from the calculated ahead values
            float aheadLR = ahead_values[0]/abs(ahead_values[0]);
            // calculating angular velocity based on angular ahead value
            float final_ahead_ang_vel = aheadLR * linValue(config.max_ang_velocity, 0.0, 2 * M_PI, abs(ahead_values[0]));
            // calculating linear value based on angular ahead value
            float ahead_lin_vel = gaussValue(config.max_lin_velocity, 2 * M_PI, abs(ahead_values[0]));
            float final_ahead_lin_vel;
            if (abs(ahead_values[0]) < 0.6) {
                // in case current vertex is a lethal vertex
                if (ahead_values[1] != std::numeric_limits<float>::max()) {
                    // basic linear velocity depending on angle difference between robot pose and plan and the cost at position
                    float lin_ahead_by_cost = linValue(config.max_lin_velocity/10,  0.0, 2.0, ahead_values[1]);
                    ahead_lin_vel -= lin_ahead_by_cost;
                    if (ahead_lin_vel < 0.0) {
                        ahead_lin_vel = 0.0;
                    } else if (ahead_lin_vel > config.max_lin_velocity) {
                        ahead_lin_vel = config.max_lin_velocity;
                    }
                }
            }
            final_ahead_lin_vel = ahead_lin_vel;

            // calculating the velocities by proportionally, combining the look ahead velocity with the velocity (without look ahead)
            final_ang_vel = (1.0-config.ahead_amount) * final_ang_vel + config.ahead_amount * final_ahead_ang_vel;
            final_lin_vel = (1.0-config.ahead_amount) * final_lin_vel + config.ahead_amount * final_ahead_lin_vel;


            // output: AHEAD angle publishing
            std_msgs::Float32 aheadAngle32;
            aheadAngle32.data = abs(ahead_values[0]) * 180 / M_PI;
            ahead_angle_pub.publish(aheadAngle32);
            // output: AHEAD angle publishing
            std_msgs::Float32 aheadCost32;
            aheadCost32.data = ahead_values[1];
            ahead_cost_pub.publish(aheadCost32);
        }

        // TODO check if fadingFactor makes sense for mesh gradient use
        final_lin_vel = final_lin_vel * fadingFactor(pose);
        // store new velocity to use as previous velocity
        set_linear_velocity = final_lin_vel;

        return {final_ang_vel, final_lin_vel};
    }

    void MeshController::reconfigureCallback(mesh_controller::MeshControllerConfig& cfg, uint32_t level)
    {
        ROS_INFO("Value configuration for mesh controller.");

        if (first_config)
        {
            config = cfg;
            first_config = false;
        }

        config = cfg;
    }

    bool  MeshController::initialize(
            const std::string& plugin_name,
            const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
            const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr){

        goalSet = false;
        name = plugin_name;
        private_nh = ros::NodeHandle("~/"+name);

        ROS_INFO_STREAM("Namespace of the controller: " << private_nh.getNamespace());
        // all for mesh plan
        map_ptr = mesh_map_ptr;


        // for PID
        // initialize integral error for PID
        int_dis_error = 0.0;
        int_dir_error = 0.0;


        set_linear_velocity = 0.0;


        reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig> > (
                new dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>(private_nh));

        config_callback = boost::bind(&MeshController::reconfigureCallback, this, _1, _2);
        reconfigure_server_ptr->setCallback(config_callback);

        angle_pub = private_nh.advertise<std_msgs::Float32>("current_angle", 1);

        ahead_angle_pub = private_nh.advertise<std_msgs::Float32>("ahead_angle", 1);
        ahead_cost_pub = private_nh.advertise<std_msgs::Float32>("ahead_cost", 1);


        return true;

    }
} /* namespace mesh_controller */
