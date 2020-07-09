#ifndef GAZEBO_PHYSICS_PLUGIN_HH
#define GAZEBO_PHYSICS_PLUGIN_HH

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>
#include <tinyxml.h>


// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo_ext_msgs/SetLinkMaterial.h"

namespace gazebo
{

class GazeboTexturePlugin : public VisualPlugin
{
  public:
  /// \brief Constructor
  GazeboTexturePlugin();

  /// \brief Destructor
  virtual ~GazeboTexturePlugin();
  void Update();

  // Documentation inherited
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

 /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  ros::NodeHandle* rosnode_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  boost::mutex lock_;

  ros::ServiceServer set_material_service_;
  bool setLinkMaterial(gazebo_ext_msgs::SetLinkMaterial::Request &req,
                                     gazebo_ext_msgs::SetLinkMaterial::Response &res);
  std::string getRandomMaterial();

  /// \brief A pointer to the visual
  rendering::VisualPtr _visual;

  /// \brief A pointer to the Gazebo node
  transport::NodePtr gzNode;
  gazebo::event::ConnectionPtr update_connection;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  void QueueThread();
  boost::thread callback_queue_thread_;
};
}
#endif