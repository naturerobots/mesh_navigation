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

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/util/Meap.hpp>
#include <mbf_msgs/ExePathResult.h>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <mbf_utility/exe_path_exception.h>

PLUGINLIB_EXPORT_CLASS(mesh_controller::MeshController,
                       mbf_mesh_core::MeshController);

namespace mesh_controller {

MeshController::MeshController() {}

MeshController::~MeshController() {}

uint32_t MeshController::computeVelocityCommands(
    const geometry_msgs::PoseStamped &pose,
    const geometry_msgs::TwistStamped &velocity,
    geometry_msgs::TwistStamped &cmd_vel, std::string &message) {

  current_pose = pose.pose;
  cancel_requested = false;

  // check if a path has been calculated and given
  if (current_plan.empty()) {
    return mbf_msgs::GetPathResult::EMPTY_PATH;
  } else if (!goalSet) {
    goal = current_plan.back();
    goalSet = true;
  }

  float max_radius_dist = 0.1;
  float dist;

  // set current face
  mesh_map::Vector position = poseToPositionVector(pose);
  std::array<float, 3> barycentric_coords;

  if (!current_face)
    ROS_WARN_STREAM("Current face not set.");
  if (!current_face && !map_ptr->searchContainingFace(
                           position, current_face, barycentric_coords, 0.3)) {
    return mbf_msgs::ExePathResult::OUT_OF_MAP;
  }

  lvr2::FaceHandle face = current_face.unwrap();
  map_ptr->publishDebugFace(face, mesh_map::color(1, 1, 1), "current_face");

  map_ptr->publishDebugPoint(position, mesh_map::color(1, 1, 1), "robot_pose");

  if ((mesh_map::projectedBarycentricCoords(
           position,
           map_ptr->mesh_ptr->getVertexPositionsOfFace(current_face.unwrap()),
           barycentric_coords, dist) &&
       dist < 0.3)) {

    map_ptr->publishDebugPoint(position, mesh_map::color(0, 0, 1),
                               "current_pos");

  } else if (map_ptr->searchNeighbourFaces(position, face, barycentric_coords,
                                           max_radius_dist, 0.1)) {
    // update current_face
    current_face = face;
    map_ptr->publishDebugFace(face, mesh_map::color(1, 0.5, 0),
                              "search_neighbour_face");
    map_ptr->publishDebugPoint(position, mesh_map::color(0, 0, 1),
                               "search_neighbour_pos");
  } else {
    return mbf_msgs::ExePathResult::OUT_OF_MAP;
  }

  const auto &vertex_positions =
      map_ptr->mesh_ptr->getVertexPositionsOfFace(face);
  const auto &vertex_handles = map_ptr->mesh_ptr->getVerticesOfFace(face);

  // projected position onto the surface.
  // position = mesh_map::linearCombineBarycentricCoords(vertex_positions,
  // barycentric_coords);

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

  // variable that contains the planned / supposed orientation of the robot at
  // the current position
  mesh_map::Vector supposed_heading;
  if (!config.useMeshGradient) {
    // use supposed orientation from calculated path
    supposed_heading = poseToDirectionVector(plan_position);
  } else {
    const auto &opt_dir =
        map_ptr->directionAtPosition(vector_map, vertex_handles, barycentric_coords);

    if (opt_dir) {
      supposed_heading = opt_dir.get().normalized();
      if(!std::isfinite(supposed_heading.length())){

      }

      ROS_INFO_STREAM("Supposed heading:" << supposed_heading);
    } else {
      map_ptr->publishDebugFace(face, mesh_map::color(0.3, 0.4, 0),
                                "no_directions");
      ROS_ERROR_STREAM("Could not access vector field for the given face!");
      return mbf_msgs::ExePathResult::FAILURE;
    }
  }

  float cost = map_ptr->costAtPosition(vertex_handles, barycentric_coords);

  // variable to store the new angular and linear velocities
  std::vector<float> values(2);

  try
  {
    // determine values via naive controller
    values = naiveControl(pose, supposed_heading, cost);
  }
  catch(mbf_utility::ExePathException e)
  {
    ROS_ERROR_STREAM("Mbf Exe Path Exception:" << e.what());
    return e.outcome;
  }

  if (values[1] == std::numeric_limits<float>::max()) {
    ROS_ERROR_STREAM("Mesh controller calculation failed!");
    return mbf_msgs::GetPathResult::FAILURE;
  }
  // set velocities

  // TODO max vel
  cmd_vel.twist.angular.z = values[0] * config.ang_vel_factor;
  cmd_vel.twist.linear.x = values[1] * config.lin_vel_factor;

  if (cancel_requested) {
    ROS_WARN_STREAM("Mesh controller will be canceled!");
    return mbf_msgs::ExePathResult::CANCELED;
  }
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool MeshController::isGoalReached(double dist_tolerance,
                                   double angle_tolerance) {
  // TODO compute the remaining distance on path or vector field
  tf::Pose robot_pose, goal_pose;
  tf::poseMsgToTF(goal.pose, goal_pose);
  tf::poseMsgToTF(current_pose, robot_pose);
  goal_distance = robot_pose.getOrigin().distance(goal_pose.getOrigin());

  return goal_distance <= static_cast<float>(dist_tolerance) &&
         angle <= static_cast<float>(angle_tolerance);
}

bool MeshController::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan)

{
  // checks if the given vector contains the plan
  if (!plan.empty()) {

    map_ptr->publishDebugPoint(poseToPositionVector(plan.front()),
                               mesh_map::color(0, 1, 0), "plan_start");
    map_ptr->publishDebugPoint(poseToPositionVector(plan.back()),
                               mesh_map::color(1, 0, 0), "plan_goal");

    // assign given plan to current_plan variable to make it usable for
    // navigation
    current_plan = plan;
    // erase first plan pose as it (usually) contains a unuseful direction
    current_plan.erase(current_plan.begin());
    // set goal according to plan
    goal = current_plan.back();
    goalSet = true;
    current_pose = current_plan.front().pose;

    // reset current and ahead face
    current_face = lvr2::OptionalFaceHandle();
    mesh_ahead_face = lvr2::OptionalFaceHandle();

    set_linear_velocity = 0;

    // set initial distance as infinite as indicator for later initializations
    initial_dist = std::numeric_limits<float>::max();
    // set the variable vector_map with the current vector map
    vector_map = map_ptr->getVectorMap();
    return true;
  } else {
    return false;
  }
}

bool MeshController::cancel() {
  ROS_INFO_STREAM("The MeshController has been requested to cancel!");
  cancel_requested = true;
  return true;
}

float MeshController::fadingFactor(const geometry_msgs::PoseStamped &pose) {
  // calculating the fading factor when using the path for naviagation
  if (!config.useMeshGradient) {
    float dist = 0.0;
    // calculate the distance between neighbouring plan positions to get the
    // whole path length
    if (initial_dist == std::numeric_limits<float>::max()) {
      // transform the first pose of plan to a tf position vector
      tf::Vector3 tf_start_vec;
      geometry_msgs::PoseStamped start_pose = current_plan.at(0);
      mesh_map::Vector start_vec = poseToPositionVector(start_pose);
      tf_start_vec = {start_vec.x, start_vec.y, start_vec.z};

      tf::Vector3 tf_next_vec;
      float update_dist = 0.0;

      for (int i = 1; i < current_plan.size(); i++) {
        // get the next pose of the plan and transform it to a tf position
        // vector
        geometry_msgs::PoseStamped next_pose = current_plan.at(i);
        mesh_map::Vector next_vec = poseToPositionVector(next_pose);
        tf_next_vec = {next_vec.x, next_vec.y, next_vec.z};
        // calculate the distance between the two vectors
        update_dist = update_dist + tf_start_vec.distance(tf_next_vec);

        // overwrite start position with next position to calculate the
        // difference between the next and next-next position in next step
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
      // overwrites start position with next position to calculate the
      // difference between the next and next-next position in next step
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
      // returns a factor slowly decreasing to 0 the end of the full velocity
      // towards the goal position
      last_fading = (initial_dist - dist) / config.fading;
      return last_fading;
    }
    // for the part of the path when the velocity does not have to be influenced
    // by a changing factor
    last_fading = 1.0;
    return last_fading;
  }
  // for mesh gradient use
  else {
    // When the mesh gradient is used only the beeline can be used to ESTIMATE
    // the distance between the start position and the goal as no knowledge
    // about the path that will be travelled is available. If this should be
    // more accurate than a a path would have to be calculated which is not the
    // intention when using the mesh gradient for navigation.
    if (initial_dist == std::numeric_limits<float>::max()) {
      // the position from which the distance is calculated is the first pose of
      // the robot
      geometry_msgs::PoseStamped start_pose = current_plan.at(0);
      mesh_map::Vector start_vec = poseToPositionVector(start_pose);
      mesh_map::Vector goal_vec = poseToPositionVector(goal);

      initial_dist = euclideanDistance(start_vec, goal_vec);
    }

    // calculate the distance between the current robot position and the start
    // position as well as the distance between the current robot position and
    // the goal position
    geometry_msgs::PoseStamped start_pose = current_plan.at(0);
    mesh_map::Vector start_vec = poseToPositionVector(start_pose);
    mesh_map::Vector goal_vec = poseToPositionVector(goal);
    mesh_map::Vector current_vec = poseToPositionVector(pose);

    float dist_start = euclideanDistance(start_vec, current_vec);
    float dist_goal = euclideanDistance(current_vec, goal_vec);
    // set the global variable goal_dist_mesh to be able to check if the goal is
    // reached
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
      // returns a factor slowly decreasing to 0 the end of the full velocity
      // towards the goal position
      last_fading = 1 - (dist_goal / config.fading);
      return last_fading;
    }
    // for the part of the path when the velocity does not have to be influenced
    // by a changing factor
    last_fading = 1.0;
    return last_fading;
  }
}

mesh_map::Vector
MeshController::poseToDirectionVector(const geometry_msgs::PoseStamped &pose) {
  // define tf Pose for later assignment
  tf::Stamped<tf::Pose> tfPose;
  // transform pose to tf:Pose
  poseStampedMsgToTF(pose, tfPose);

  // get x as tf:Vector of transformed pose
  tf::Vector3 v = tfPose.getBasis() * tf::Vector3(1, 0, 0);
  // transform tf:Vector in mesh_map Vector and return
  return mesh_map::Vector(v.x(), v.y(), v.z());
}

mesh_map::Vector
MeshController::poseToPositionVector(const geometry_msgs::PoseStamped &pose) {
  return {(float)pose.pose.position.x, (float)pose.pose.position.y,
          (float)pose.pose.position.z};
}

float MeshController::angleBetweenVectors(mesh_map::Vector robot_heading,
                                          mesh_map::Vector planned_heading) {
  tf::Vector3 tf_robot(
      robot_heading.x,
      robot_heading.y,
      robot_heading.z);
  tf::Vector3 tf_planned(
      planned_heading.x,
      planned_heading.y,
      planned_heading.z);
  return tf_robot.angle(tf_planned);
}

float MeshController::linValue(float max_hight, float x_axis, float max_width,
                               float value) {
  // checking higher and lower boarders
  if (value > max_width / 2) {
    return max_hight;
  } else if (value < -max_width / 2) {
    return -max_hight;
  }
  // calculating the incline of the linear function
  float incline = max_hight / (max_width / 2);
  return abs(incline * (value + x_axis));
}

float MeshController::gaussValue(float max_hight, float max_width,
                                 float value) {
  // in case value lays outside width, the function goes to zero
  if (value > max_width / 2) {
    return 0.0;
  }

  //  calculating the standard deviation given the max_width
  // based on the fact that 99.7% of area lies between mü-3*sigma and mü+3*sigma
  float std_dev = pow(-max_width / 6, 2);

  // calculating y value of given normal distribution
  // stretched to max_hight and desired width
  float y_value = max_hight * 1 / (sqrtf(2 * M_PI * std_dev)) *
                  pow(E, (-pow(value, 2) * pow((2 * std_dev), 2)));

  return y_value;
}

float MeshController::direction(mesh_map::Vector &robot_heading,
                                mesh_map::Vector &planned_heading) {

  tf::Vector3 tf_robot(robot_heading.x, robot_heading.y, robot_heading.z);
  tf::Vector3 tf_planned(planned_heading.x, planned_heading.y,
                         planned_heading.z);

https: // www.gamedev.net/forums/topic/508445-left-or-right-direction/
  tf::Vector3 tf_cross_prod = tf_robot.cross(tf_planned);

  tf::Vector3 tf_up = {0, 0, -1};

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

bool MeshController::offPlan(const geometry_msgs::PoseStamped &robot_pose) {
  // transform the first pose of plan to a tf position vector
  mesh_map::Vector robot_vec = poseToPositionVector(robot_pose);
  mesh_map::Vector plan_vec = poseToPositionVector(plan_position);

  if (euclideanDistance(robot_vec, plan_vec) > config.off_plan) {
    return true;
  }
  return false;
}

float MeshController::euclideanDistance(
    const geometry_msgs::PoseStamped &pose) {
  mesh_map::Vector pose_vec = poseToPositionVector(pose);
  tf::Vector3 tf_pose_vec = {pose_vec.x, pose_vec.y, pose_vec.z};

  mesh_map::Vector goal_vec = poseToPositionVector(goal);
  tf::Vector3 tf_goal_vec = {goal_vec.x, goal_vec.y, goal_vec.z};

  return tf_pose_vec.distance(tf_goal_vec);
}

float MeshController::euclideanDistance(lvr2::BaseVector<float> &current,
                                        lvr2::BaseVector<float> &planned) {
  tf::Vector3 tf_current_vec = {current.x, current.y, current.z};
  tf::Vector3 tf_planned_vec = {planned.x, planned.y, planned.z};

  return tf_current_vec.distance(tf_planned_vec);
}

uint32_t MeshController::updatePlanPos(const geometry_msgs::PoseStamped &pose,
                                       float velocity) {
  // checking if the function has been called before
  // if not - set the time for the first time
  if (last_call.isZero()) {
    last_call = ros::Time::now();
    // initialize the iterator for the planned path with zero = start position
    plan_iter = 0;
    return mbf_msgs::ExePathResult::SUCCESS;
  }
  // calculate the time between now and the last call of the function
  ros::Time now = ros::Time::now();
  ros::Duration time_delta = now - last_call;

  // determine the maximum distance that the robot could have driven between the
  // last function call and now based on its velocity reason : the faster the
  // robot, the further the robot might have travelled on the planned path
  double max_dist = velocity * time_delta.toSec();
  float min_dist = std::numeric_limits<float>::max();

  // transform the current robot pose to a position vector
  mesh_map::Vector robot_vec = poseToPositionVector(pose);

  int iter, ret_iter;
  // iterator from which to start search
  iter = plan_iter;

  float dist = 0;
  // look forward on the plan and check if another position on the plan is
  // closer to the robot
  do {
    if (cancel_requested) {
      ROS_WARN_STREAM("Cancel has been requested while looking forward on the "
                      "plan. Canceling...");
      return mbf_msgs::ExePathResult::CANCELED;
    }

    // get the position on the path
    geometry_msgs::PoseStamped iter_pose = current_plan[iter];
    // transform it to a position vector
    mesh_map::Vector iter_vec = poseToPositionVector(iter_pose);
    // calculate the distance between the robot position and the planned path
    // position
    dist = euclideanDistance(robot_vec, iter_vec);
    // if the distance is smaller than the previously calculated one, save the
    // values
    if (dist < min_dist) {
      ret_iter = iter;
      min_dist = dist;
    }
    iter++;
  }
  // repeat this until either the maximum travelled distance is reached or the
  // planned path ends
  while (dist > max_dist && iter < current_plan.size());

  // reset iterator
  iter = plan_iter;

  // look back, as above but going backwards through the plan
  do {
    if (cancel_requested) {
      ROS_WARN_STREAM("Cancel has been requested while looking backwards on "
                      "the plan. Canceling...");
      return mbf_msgs::ExePathResult::CANCELED;
    }
    geometry_msgs::PoseStamped iter_pose = current_plan[iter];
    mesh_map::Vector iter_vec = poseToPositionVector(iter_pose);
    dist = euclideanDistance(robot_vec, iter_vec);
    if (dist < min_dist) {
      ret_iter = iter;
      min_dist = dist;
    }
    iter--;
  }
  // do this until either the maximum travelled distance is or the start of the
  // planned path is reached
  while (dist > max_dist && iter >= 0);

  // update the values
  // plan iter becomes the iterator of the position with the smallest distance
  // to the robot
  plan_iter = ret_iter;
  // plan position becomes the position of the plan with the smallest distance
  // to the robot
  plan_position = current_plan[plan_iter];
  last_call = now;
  return mbf_msgs::ExePathResult::SUCCESS;
}

std::vector<float>
MeshController::lookAhead(const geometry_msgs::PoseStamped &pose,
                          float velocity) {
  mesh_map::Vector robot_heading = poseToDirectionVector(pose);
  mesh_map::Vector robot_position = poseToPositionVector(pose);

  // initialize time of last look ahead call when function has not been called
  // before
  if (last_lookahead_call.isZero()) {
    last_lookahead_call = ros::Time::now();
    return {std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
  }
  // determine the time that has passed since the last look ahead
  ros::Time now = ros::Time::now();
  ros::Duration time_delta = now - last_lookahead_call;
  last_lookahead_call = now;

  // calculate the maximum distance the robot could have travelled since the
  // last function call because: the faster the robot, the further the distance
  // that may be travelled and therefore the look ahead

  double max_travelled_dist = velocity * time_delta.toSec();

  // select how far to look ahead depending on the max travelled distance
  int steps;
  if (!config.useMeshGradient) {
    // calculates approximately how many elements of the path have to be called
    // and travelled along to reach the maximum distance that could be travelled
    // with the given velocity
    steps = (int)linValue(current_plan.size(), 0.0, 2 * initial_dist,
                          max_travelled_dist) -
            plan_iter;
    ROS_ERROR_STREAM("1 Steps:" << steps);
  } else {
    // calculates how many steps ahead have to be checked when travelling in the
    // current robot direction with the given velocity by dividing it through
    // the step size that is later used therefore the look ahead will be as far
    // as the maximum travelled distance
    steps = (int)(max_travelled_dist / 0.03);
    ROS_ERROR_STREAM("2 Steps:" << steps << "max travel dist:" << max_travelled_dist);
    ROS_ERROR_STREAM("velocity:" << velocity);
    ROS_ERROR_STREAM("time delta:" << time_delta.toSec());
  }

  if (steps == 0) {
    // no look ahead when there is no linear velocity
    // values have to be dealt with after function call
    return {std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
  }

  // variable to add how many times the cost could not be calculated and
  // therefore not added up
  int missed_steps = 0;
  // variable to store on which position of the plan a lethal vertex will be
  // first encountered
  int lethal_step = 0;
  // variables to accumulate the cost and direction values of the future
  // positions
  float accum_cost = 0.0;
  float accum_turn = 0.0;
  // initialize the face ahead with the current face

  mesh_map::Vector ahead_position_vec = robot_position;
  mesh_map::Vector ahead_direction_vec;
  float ahead_cost = -1;

  ROS_ERROR_STREAM("4. Steps:" << steps);

  lvr2::FaceHandle ahead_face = current_face.unwrap();
  // adds up cost and angles of all steps ahead
  for (int i = 0; i <= steps; i++) {
    // look ahead when using the planned path for navigation reference
    ahead_cost = -1;
    if (!config.useMeshGradient) {
      // in case look ahead extends planned path
      if ((plan_iter + i) >= current_plan.size()) {
        steps = i;
        ROS_ERROR_STREAM("3. Steps:" << steps);
        break;
      }

      // gets the pose of the ahead position
      geometry_msgs::PoseStamped pose_ahead = current_plan[plan_iter + i];
      // converts the pose to a position vector
      ahead_position_vec = poseToPositionVector(pose_ahead);
      // converts the pose into a direction vector
      ahead_direction_vec = poseToDirectionVector(pose_ahead);
      // finds the face which contains the current ahead face
      // find cost of the future position
      const auto &vertex_handles =
          map_ptr->mesh_ptr->getVerticesOfFace(ahead_face);
      const auto &vertex_positions =
          map_ptr->mesh_ptr->getVertexPositionsOfFace(ahead_face);
      std::array<float, 3> barycentric_coords;
      float dist;

      if (mesh_map::projectedBarycentricCoords(
              ahead_position_vec, vertex_positions, barycentric_coords, dist) ||
          map_ptr->searchNeighbourFaces(ahead_position_vec, ahead_face,
                                        barycentric_coords, 0.05, 0.4)) {
        ahead_cost =
            map_ptr->costAtPosition(vertex_handles, barycentric_coords);
        if(ahead_cost == -1) ROS_ERROR_STREAM("Could not access cost!");
      }
    }
    // look ahead for using mesh gradient for navigation reference
    else {
      const auto &vertex_positions =
          map_ptr->mesh_ptr->getVertexPositionsOfFace(ahead_face);
      std::array<float, 3> barycentric_coords;
      float dist;
      if (mesh_map::projectedBarycentricCoords(
              ahead_position_vec, vertex_positions, barycentric_coords, dist) ||
          map_ptr->searchNeighbourFaces(ahead_position_vec, ahead_face,
                                        barycentric_coords, 0.05, 0.4)) {

        const auto &vertex_handles =
            map_ptr->mesh_ptr->getVerticesOfFace(ahead_face);

        const auto &opt_dir =
            map_ptr->directionAtPosition(vector_map, vertex_handles, barycentric_coords);
        if (opt_dir) {
          ahead_direction_vec = opt_dir.get().normalized() * 0.03;
          ahead_position_vec += ahead_direction_vec;
          ahead_cost =
              map_ptr->costAtPosition(vertex_handles, barycentric_coords);
          if(ahead_cost == -1) ROS_ERROR_STREAM("Could not access cost!!!");
        } else {
          ROS_ERROR_STREAM("No direction at position!");
          break;
        }
      }
    }

    if (ahead_cost == std::numeric_limits<float>::infinity() &&
        lethal_step == 0) {
      ROS_INFO_STREAM("lethal vertex " << i);
      lethal_step = i;
    } else if (ahead_cost == -1.0) {
      ROS_INFO("cost could not be accessed");
      // cost could not be accessed
      // CAUTION could lead to division by zero
      missed_steps += 1;
    } else {
      // get direction difference between current position and next position
      float future_turn =
          angleBetweenVectors(robot_heading, ahead_direction_vec);
      // to determine in which direction to turn in future (neg for left, pos
      // for right)
      float leftRight = direction(robot_heading, ahead_direction_vec);
      // accumulate cost and angle
      accum_cost += ahead_cost;
      accum_turn += (future_turn * leftRight);
    }

    if (cancel_requested) {
      return {0, 0};
    }
  }

  if ((steps - missed_steps) <= 0 || steps <= 0) {
    ROS_ERROR_STREAM("Missed steps:" << missed_steps << " Steps:" << steps);
    throw mbf_utility::ExePathException(mbf_msgs::ExePathResult::MAP_ERROR);
  }
  //  take averages of future values
  float av_turn = accum_turn / (steps - missed_steps);
  float av_cost = accum_cost / (steps - missed_steps);

  return {av_turn, av_cost};
}

std::vector<float>
MeshController::naiveControl(const geometry_msgs::PoseStamped &pose,
                             mesh_map::Vector supposed_dir, const float &cost) {
  mesh_map::Vector dir_vec = poseToDirectionVector(pose);
  mesh_map::Vector position_vec = poseToPositionVector(pose);

  // ANGULAR MOVEMENT
  // calculate angle between orientation vectors
  // angle will never be negative and smaller or equal to pi
  angle = angleBetweenVectors(dir_vec, supposed_dir);

  ROS_INFO_STREAM("dir vec:" << dir_vec);
  ROS_INFO_STREAM("sup vec:" << supposed_dir);

  // output: angle publishing
  std_msgs::Float32 angle32;
  angle32.data = angle * 180 / M_PI;
  angle_pub.publish(angle32);

  // to determine in which direction to turn (neg for left, pos for right)
  float leftRight = direction(dir_vec, supposed_dir);

  // calculate a direction velocity depending on the turn direction and
  // difference angle
  float final_ang_vel =
      leftRight * linValue(config.max_ang_velocity, 0.0, 2 * M_PI, angle);

  // LINEAR movement
  ROS_ERROR_STREAM("angle: "<< angle);
  float lin_vel_by_ang = gaussValue(config.max_lin_velocity, 2 * M_PI, angle);
  float final_lin_vel;

  ROS_ERROR_STREAM("lin value by angle: "<< lin_vel_by_ang);

  // check the size of the angle. If it is not more than about 35 degrees,
  // integrate position costs to linear velocity
  if (angle < 0.6) {
    float cost_lin_vel = cost;
    // in case current vertex is a lethal vertex
    if (cost_lin_vel != std::numeric_limits<float>::max()) {
      // basic linear velocity depending on angle difference between robot pose
      // and plan and the cost at position
      float lin_factor_by_cost =
          linValue(config.max_lin_velocity / 10, 0.0, 2.0, cost_lin_vel);
      lin_vel_by_ang -= lin_factor_by_cost;
      if (lin_vel_by_ang < 0.0) {
        lin_vel_by_ang = 0.0;
      } else if (lin_vel_by_ang > config.max_lin_velocity) {
        lin_vel_by_ang = config.max_lin_velocity;
      }
    }
  }
  final_lin_vel = lin_vel_by_ang;
  ROS_ERROR_STREAM("Final lin velocity: "<< final_lin_vel);


  // ADDITIONAL factors
  // look ahead
  std::vector<float> ahead_values = lookAhead(pose, set_linear_velocity);

  if (ahead_values[1] != std::numeric_limits<float>::max()) {
    // get the direction factor from the calculated ahead values
    float aheadLR = ahead_values[0] / abs(ahead_values[0]);
    // calculating angular velocity based on angular ahead value
    float final_ahead_ang_vel =
        aheadLR *
        linValue(config.max_ang_velocity, 0.0, 2 * M_PI, abs(ahead_values[0]));
    // calculating linear value based on angular ahead value
    float ahead_lin_vel =
        gaussValue(config.max_lin_velocity, 2 * M_PI, abs(ahead_values[0]));
    ROS_ERROR_STREAM("Final lin velocity after gauss: "<< final_lin_vel);
    float final_ahead_lin_vel;
    if (abs(ahead_values[0]) < 0.6) {
      // basic linear velocity depending on angle difference between robot pose
      // and plan and the cost at position
      float lin_ahead_by_cost =
          linValue(config.max_lin_velocity / 10, 0.0, 2.0, ahead_values[1]);
      ahead_lin_vel -= lin_ahead_by_cost;
      if (ahead_lin_vel < 0.0) {
        ahead_lin_vel = 0.0;
      } else if (ahead_lin_vel > config.max_lin_velocity) {
        ahead_lin_vel = config.max_lin_velocity;
      }
    }
    final_ahead_lin_vel = ahead_lin_vel;

    // calculating the velocities by proportionally, combining the look ahead
    // velocity with the velocity (without look ahead)
    final_ang_vel = (1.0 - config.ahead_amount) * final_ang_vel +
                    config.ahead_amount * final_ahead_ang_vel;
    final_lin_vel = (1.0 - config.ahead_amount) * final_lin_vel +
                    config.ahead_amount * final_ahead_lin_vel;
    ROS_ERROR_STREAM("Final lin velocity: "<< final_lin_vel << " final ahead lin vel:" << final_ahead_lin_vel);

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

void MeshController::reconfigureCallback(
    mesh_controller::MeshControllerConfig &cfg, uint32_t level) {
  ROS_INFO("Value configuration for mesh controller.");

  if (first_config) {
    config = cfg;
    first_config = false;
  }

  config = cfg;
}

bool MeshController::initialize(
    const std::string &plugin_name,
    const boost::shared_ptr<tf2_ros::Buffer> &tf_ptr,
    const boost::shared_ptr<mesh_map::MeshMap> &mesh_map_ptr) {

  goalSet = false;
  name = plugin_name;
  private_nh = ros::NodeHandle("~/" + name);

  ROS_INFO_STREAM("Namespace of the controller: " << private_nh.getNamespace());
  // all for mesh plan
  map_ptr = mesh_map_ptr;

  // for PID
  // initialize integral error for PID
  int_dis_error = 0.0;
  int_dir_error = 0.0;

  set_linear_velocity = 0.0;

  reconfigure_server_ptr = boost::shared_ptr<
      dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>>(
      new dynamic_reconfigure::Server<mesh_controller::MeshControllerConfig>(
          private_nh));

  config_callback =
      boost::bind(&MeshController::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

  angle_pub = private_nh.advertise<std_msgs::Float32>("current_angle", 1);
  position_pub =
      private_nh.advertise<geometry_msgs::PointStamped>("current_position", 1);

  ahead_angle_pub = private_nh.advertise<std_msgs::Float32>("ahead_angle", 1);
  ahead_cost_pub = private_nh.advertise<std_msgs::Float32>("ahead_cost", 1);

  return true;
}
} /* namespace mesh_controller */
