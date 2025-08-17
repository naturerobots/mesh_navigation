#include <mesh_layers/obstacle_layer.h>
#include <mesh_map/mesh_map.h>
#include <mesh_map/timer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace mesh_layers
{

std::optional<rclcpp::QoS> get_qos_profile_from_string(const std::string& str)
{
  if ("Reliable" == str)
  {
    return rclcpp::QoS(1).reliable();
  }
  else if ("BestEffort")
  {
    return rclcpp::QoS(1).best_effort();
  }

  return std::nullopt;
}

bool ObstacleLayer::initialize()
{
  RCLCPP_DEBUG(get_logger(), "Initializing 'ObstacleLayer' with name '%s'", name().c_str());

  auto map = map_ptr_.lock();

  // Read parameters
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "robot_height";
    desc.description = "The height of the robot in meter. "
    "Obstacles with a larger distance to the surface along the vertical axis are not added to the cost map. "
    "This allows the robot to pass below overhanging obstacles like tables (if it can fit below the overhang). "
    "You should add a safety margin to the robot height!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    config_.robot_height = node_->declare_parameter(desc.name, config_.robot_height, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "max_obstacle_dist";
    desc.description = "The maximum distance to the sensor origin ([0.0, 0.0, 0.0] in the reference frame of the input point cloud!) an obstacle point may have. "
    "Obstacles with larger distances to the (sensor) origin are not processed and added to the map. "
    "Limiting the sensor range reduces the update latency, especially with lidar sensors with high ranges.";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    config_.max_obstacle_dist = node_->declare_parameter(desc.name, config_.max_obstacle_dist, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "topic";
    desc.description = "The ROS topic to subscribe to. The message type must be PointCloud2! "
      "This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    config_.topic = node_->declare_parameter<std::string>(desc.name, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "qos";
    desc.description = "The QoS settings to use when subscribing to the ROS topic. Options are 'Reliable' and 'BestEffort'. "
      "This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    const std::string qos_str = node_->declare_parameter<std::string>(desc.name, "Reliable", desc);
    if (auto opt = get_qos_profile_from_string(qos_str))
    {
      config_.qos = opt.value();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid 'qos' parameter '%s'! Options are 'Reliable' and 'BestEffort'", qos_str.c_str());
      return false;
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "tf_tolerance";
    desc.description = "The time to wait for transforms in seconds.";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    const double tf_tolerance = node_->declare_parameter(desc.name, config_.tf_tolerance.seconds(), desc);
    config_.tf_tolerance = rclcpp::Duration::from_seconds(tf_tolerance);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "down_axis";
    desc.description = "The vector to use when projecting obstacle points to the surface. "
      "Defaults to [0.0, 0.0, -1.0]. This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    const auto dir = node_->declare_parameter<std::vector<double>>(desc.name, {0.0, 0.0, -1.0}, desc);
    // Check for exactly 3 coords!
    if (3 != dir.size())
    {
      RCLCPP_ERROR(get_logger(), "Invalid parameter value for 'down_axis'! Must be exactly 3 values!");
      return false;
    }
    config_.down_axis = lvr2::Vector3f(dir[0], dir[1], dir[2]).normalized();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = layer_namespace_ + '.' + "axis_frame";
    desc.description = "The reference frame of the 'down_axis' parameter. Defaults to the value of the robot_frame parameter. "
      "This parameter is not reconfigurable at runtime!";
    desc.type =rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

    // Default to the 'robot_frame' parameter. This should always have a value
    const std::string robot_frame = node_->get_parameter("robot_frame").as_string();
    config_.axis_frame_id = node_->declare_parameter(desc.name, robot_frame, desc);
  }

  // Support reconfiguration of parameters at runtime
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
    std::bind(&ObstacleLayer::reconfigureCallback, this, std::placeholders::_1)
  );

  // Setup a callback group for multithreaded ros callback execution
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  // Initialize ROS Subscriber
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.topic,
    config_.qos,
    std::bind(&ObstacleLayer::processPointCloud, this, std::placeholders::_1),
    options
  );

  return true;
}


void ObstacleLayer::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  RCLCPP_DEBUG(get_logger(), "Processing point cloud with %u points", msg->width * msg->height);
  mesh_map::LayerTimer::TimePoint t0 = mesh_map::LayerTimer::Clock::now();

  const auto map = map_ptr_.lock();
  if (nullptr == map)
  {
    RCLCPP_ERROR(get_logger(), "Could not update cost map: Failed to lock map_ptr_");
    return;
  }

  const auto raycaster = map->raycaster();
  if (nullptr == map)
  {
    RCLCPP_ERROR(get_logger(), "Could not get raycasting interface from map! nullptr == raycaster");
    return;
  }

  mesh_map::LayerTimer::TimePoint t1 = mesh_map::LayerTimer::Clock::now();

  // Get transform from message to map frame
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = map->tf2Buffer().lookupTransform(
      map->mapFrame(),
      msg->header.frame_id,
      msg->header.stamp,
      config_.tf_tolerance
    );
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to lookup transform from %s -> %s: %s",
      msg->header.frame_id.c_str(), map->mapFrame().c_str(), ex.what()
    );
    return;
  }
  RCLCPP_DEBUG(get_logger(), "Got transform from %s -> %s", msg->header.frame_id.c_str(), map->mapFrame().c_str());

  // Convert TF2 transform to Eigen Isometry3
  Eigen::Quaternionf quat(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
  quat.normalize();
  Eigen::Isometry3f eigen_tf(
    Eigen::Translation3f(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z) * quat
  );

  // Transform the axis from the axis_frame_id to map
  lvr2::Vector3f transformed_axis = config_.down_axis;
  try
  {
    geometry_msgs::msg::TransformStamped axis_tf = map->tf2Buffer().lookupTransform(
      map->mapFrame(),
      config_.axis_frame_id,
      msg->header.stamp,
      config_.tf_tolerance
    );

    // Rotate the down_axis vector
    Eigen::Quaternionf quat(axis_tf.transform.rotation.w, axis_tf.transform.rotation.x, axis_tf.transform.rotation.y, axis_tf.transform.rotation.z);
    quat.normalize();
    transformed_axis = quat * config_.down_axis;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to lookup transform from %s -> %s: %s",
      config_.axis_frame_id.c_str(), map->mapFrame().c_str(), ex.what()
    );
    return;
  }
  RCLCPP_DEBUG(
    get_logger(), "Transformed down_axis from %s -> %s; Axis: [%f.2, %f.2, %f.2]",
    config_.axis_frame_id.c_str(), map->mapFrame().c_str(), transformed_axis.x(), transformed_axis.y(), transformed_axis.z()
  );

  // Convert to lvr2 representation and filter by distance!
  std::vector<lvr2::Vector3f> origins;
  sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
  for(;x_it != x_it.end() && y_it != y_it.end() && z_it != z_it.end(); ++x_it, ++y_it, ++z_it)
  {
    lvr2::Vector3f p(*x_it, *y_it, *z_it);
    if (p.norm() <= config_.max_obstacle_dist)
    {
      // Transform to map frame
      origins.push_back(eigen_tf * p);
    }
  }

  // Allocate buffers
  std::vector<lvr2::Vector3f> dirs(origins.size(), transformed_axis);
  std::vector<mesh_map::MeshMap::RayCastResult> results(origins.size());
  std::vector<uint8_t> hits(origins.size());

  RCLCPP_DEBUG(
    get_logger(),
    "Casting %lu rays; Robot height: %f; Max obstacle dist: %f",
    origins.size(), config_.robot_height, config_.max_obstacle_dist
  );
  raycaster->castRays(origins, dirs, results, hits);

  // Update the lethal vertices and cost map
  // Create new cost map and lethal set
  lvr2::SparseVertexMap<float> costs;
  std::set<lvr2::VertexHandle> lethals;
  for (size_t idx = 0; idx < hits.size(); idx++)
  {
    if (hits[idx] && results[idx].dist <= config_.robot_height)
    {
      const lvr2::FaceHandle face(results[idx].face_id);
      for (const auto vertex: map->mesh()->getVerticesOfFace(face))
      {
        costs.insert(vertex, std::numeric_limits<float>::infinity());
        lethals.insert(vertex);
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Found %lu lethal vertices", lethals.size());

  // Find all vertices that are no longer considered lethal
  std::set<lvr2::VertexHandle> no_longer_lethal;
  std::set_difference(
    lethals_.begin(), lethals_.end(),
    lethals.begin(), lethals.end(),
    std::inserter(no_longer_lethal, no_longer_lethal.end())
  );

  // All vertices that have changed
  std::set<lvr2::VertexHandle> changed;
  std::set_symmetric_difference(
    lethals_.begin(), lethals_.end(),
    lethals.begin(), lethals.end(),
    std::inserter(changed, changed.end())
  );

  {
    // Update the cost map and lethal vertices
    // Aquire a write lock to prevent race conditions.
    // We use a scope here to unlock the write lock after we are done updating
    // the layer data.
    auto wlock = this->writeLock();
    costs_ = std::move(costs);
    lethals_ = std::move(lethals);
  }
  mesh_map::LayerTimer::TimePoint t2 = mesh_map::LayerTimer::Clock::now();

  RCLCPP_DEBUG(get_logger(), "calling notifyChange() with %lu changed vertices", changed.size());
  // Our write lock has to be unlocked before we call notifyChange to prevent
  // a deadlock when other layers are notified and try to read this layer!
  this->notifyChange(msg->header.stamp, changed);
  mesh_map::LayerTimer::TimePoint t3 = mesh_map::LayerTimer::Clock::now();

  // Log timing information for debug purposes, see \ref mesh_map::LayerTimer for more information
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, msg->header.stamp, t1 - t0, t2 - t1, t3 - t2);
}


rcl_interfaces::msg::SetParametersResult ObstacleLayer::reconfigureCallback(
  std::vector<rclcpp::Parameter> parameters
)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& param: parameters)
  {
    if (layer_namespace_ + ".robot_height" == param.get_name())
    {
      config_.robot_height = param.as_double();
      RCLCPP_INFO(get_logger(), "Parameter 'robot_height' reconfigured to '%f'", config_.robot_height);
    }
    else if (layer_namespace_ + ".max_obstacle_dist" == param.get_name())
    {
      config_.max_obstacle_dist = param.as_double();
      RCLCPP_INFO(get_logger(), "Parameter 'max_obstacle_dist' reconfigured to '%f'", config_.max_obstacle_dist);
    }
    else if (layer_namespace_ + ".tf_tolerance" == param.get_name())
    {
      config_.tf_tolerance = rclcpp::Duration::from_seconds(param.as_double());
      RCLCPP_INFO(get_logger(), "Parameter 'tf_tolerance' reconfigured to '%f'", config_.tf_tolerance.seconds());
    }
  }

  return res;
}

} // namespace mesh_layers

// Register the layer plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mesh_layers::ObstacleLayer, mesh_map::AbstractLayer)
