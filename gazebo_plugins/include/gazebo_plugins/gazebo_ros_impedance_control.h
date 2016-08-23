#ifndef GAZEBO_ROS_IMPEDANCE_HH
#define GAZEBO_ROS_IMPEDANCE_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosImpedanceControl Plugin XML Reference and Example

  \brief Ros Impedance Control Plugin.
  
  This is a Plugin that collects data of pose and wrench from a ROS topic and applies pose to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_impedance_control.so" name="gazebo_ros_impedance_control">
          <bodyName>box_body</bodyName>
          <topicName>box_impedance</topicName>
        </plugin>
      </gazebo>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosImpedanceControl : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosImpedanceControl();

  /// \brief Destructor
  public: virtual ~GazeboRosImpedanceControl();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateImpedanceControl();

  /// \brief call back when a message is published
  /// \param[in] _msg The Incoming ROS message representing the new force to exert.
  private: void UpdateReferencePose(const geometry_msgs::Pose::ConstPtr& _msg);
  private: void UpdateReferenceWrench(const geometry_msgs::Wrench::ConstPtr& _msg);
  private: void UpdateReferenceTwist(const geometry_msgs::Twist::ConstPtr& _msg);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber pose_sub_;
  private: ros::Subscriber wrench_sub_;
  private: ros::Subscriber twist_sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief save last_time
  private: common::Time last_time_;

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;

  /// \brief ROS frame transform name to use in the image message header.
  private: std::string frame_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  /// \brief The Link, where pose is measured and controlled
  private: std::string link_name_;

  /// \brief A pointer to the Link, where pose is measured and controlled
  private: physics::LinkPtr link_;

  /// \brief The Joint, where force/torque is measured and controlled
  private: std::string joint_name_;

  /// \brief A pointer to the Gazebo joint, where force/torque is measured and controlled
  private: physics::JointPtr joint_;

  /// \brief A pointer to the Gazebo parent link of joint
  private: physics::LinkPtr parent_link_;

  /// \brief A pointer to the Gazebo child link of joint
  private: physics::LinkPtr child_link_;

  // \brief control rate
  private: double update_rate_;

  /// \brief ROS Pose topic name inputs
  private: std::string pose_topic_name_;

  /// \brief ROS Wrench topic name inputs
  private: std::string wrench_topic_name_;

  /// \brief ROS Twist topic name inputs
  private: std::string twist_topic_name_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

  // \berief output position
  private: math::Vector3 out_position_;
  // \berief output orientation
  private: math::Quaternion out_orientation_;

  // \berief reference position
  private: math::Vector3 ref_position_;
  // \berief reference orientation
  private: math::Quaternion ref_orientation_;

  // \berief reference force
  private: math::Vector3 ref_force_;
  // \berief reference torque
  private: math::Vector3 ref_torque_;

  // \berief reference linear velocity
  private: math::Vector3 ref_linear_vel_;
  // \berief reference angular velocity
  private: math::Vector3 ref_angular_vel_;

  // \berief actual force
  private: math::Vector3 act_force_;
  // \berief actual torque
  private: math::Vector3 act_torque_;
};
/** \} */
/// @}
}
#endif
