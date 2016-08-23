#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_impedance_control.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosImpedanceControl);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosImpedanceControl::GazeboRosImpedanceControl()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosImpedanceControl::~GazeboRosImpedanceControl()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosImpedanceControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->model_ = _model;
  this->world_ = _model->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("force plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("ft_sensor plugin missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();

  this->joint_ = this->model_->GetJoint(this->joint_name_);
  if (!this->joint_)
  {
    ROS_FATAL("gazebo_ros_ft_sensor plugin error: jointName: %s does not exist\n",this->joint_name_.c_str());
    return;
  }

  this->parent_link_ = this->joint_->GetParent();
  this->child_link_ = this->joint_->GetChild();
  this->frame_name_ = this->child_link_->GetName();

  if (!_sdf->HasElement("poseTopicName"))
  {
    ROS_FATAL("force plugin missing <poseTopicName>, cannot proceed");
    return;
  }
  else
    this->pose_topic_name_ = _sdf->GetElement("poseTopicName")->Get<std::string>();

  if (!_sdf->HasElement("wrenchTopicName"))
  {
    ROS_FATAL("force plugin missing <wrenchTopicName>, cannot proceed");
    return;
  }
  else
    this->wrench_topic_name_ = _sdf->GetElement("wrenchTopicName")->Get<std::string>();

  if (!_sdf->HasElement("twistTopicName"))
  {
    ROS_FATAL("force plugin missing <twistTopicName>, cannot proceed");
    return;
  }
  else
    this->twist_topic_name_ = _sdf->GetElement("twistTopicName")->Get<std::string>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG("ft_sensor plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // Custom Callback Queue
  ros::SubscribeOptions pose_so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
    this->pose_topic_name_,1,
    boost::bind( &GazeboRosImpedanceControl::UpdateReferencePose,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->pose_sub_ = this->rosnode_->subscribe(pose_so);
  ros::SubscribeOptions wrench_so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->wrench_topic_name_,1,
    boost::bind( &GazeboRosImpedanceControl::UpdateReferenceWrench,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->wrench_sub_ = this->rosnode_->subscribe(wrench_so);
  ros::SubscribeOptions twist_so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    this->twist_topic_name_,1,
    boost::bind( &GazeboRosImpedanceControl::UpdateReferenceTwist,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->twist_sub_ = this->rosnode_->subscribe(twist_so);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosImpedanceControl::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosImpedanceControl::UpdateImpedanceControl, this));

  // disable gravity
  this->model_->SetGravityMode(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the reference
void GazeboRosImpedanceControl::UpdateReferencePose(const geometry_msgs::Pose::ConstPtr& _msg)
{
  this->ref_position_ = math::Vector3(_msg->position.x, _msg->position.y, _msg->position.z);
  this->ref_orientation_ = math::Quaternion(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
}

void GazeboRosImpedanceControl::UpdateReferenceWrench(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->ref_force_ = math::Vector3(_msg->force.x, _msg->force.y, _msg->force.z);
  this->ref_torque_ = math::Vector3(_msg->torque.x, _msg->torque.y, _msg->torque.z);
}

void GazeboRosImpedanceControl::UpdateReferenceTwist(const geometry_msgs::Twist::ConstPtr& _msg)
{
  this->ref_linear_vel_ = math::Vector3(_msg->linear.x, _msg->linear.y, _msg->linear.z);
  this->ref_angular_vel_ = math::Vector3(_msg->angular.x, _msg->angular.y, _msg->angular.z);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosImpedanceControl::UpdateImpedanceControl()
{
  // rate control
  common::Time cur_time = this->world_->GetSimTime();
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  this->lock_.lock();

  // get actual wrench
  physics::JointWrench wrench = this->joint_->GetForceTorque(0);
  this->act_force_  = wrench.body2Force;
  this->act_torque_  = wrench.body2Torque;


  // compute wrench error
  math::Vector3 force_error = act_force_ - ref_force_;
  math::Vector3 torque_error = act_torque_ - ref_torque_;

  // compute out pose
  // [ToDo]
  this->out_position_ = this->ref_position_;
  this->out_orientation_ = this->ref_orientation_;

  // apply out pose
  math::Pose out_pose(this->out_position_, this->out_orientation_);
  // this->model_->SetLinkWorldPose(out_pose, this->link_);
  // this->model_->SetLinearVel(math::Vector3(0, 0, 0));
  // this->model_->SetLinearAccel(math::Vector3(0, 0, 0));
  this->model_->SetLinearVel(this->ref_linear_vel_);
  this->model_->SetLinearAccel(math::Vector3(0, 0, 0));
  this->model_->SetAngularVel(this->ref_angular_vel_);
  this->model_->SetAngularAccel(math::Vector3(0, 0, 0));

  // debug message
  // ROS_WARN_STREAM("[time] step time: " << (cur_time-this->last_time_));
  // ROS_WARN_STREAM("[force] act: " << act_force_ << " ref: " << ref_force_ << " error: " << force_error);
  // ROS_WARN_STREAM("[torque] act: " << act_torque_ << " ref: " << ref_torque_ << " error: " << torque_error);
  // ROS_WARN_STREAM("[pose] out: " << out_pose);

  this->lock_.unlock();
  this->last_time_ = cur_time;
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosImpedanceControl::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
