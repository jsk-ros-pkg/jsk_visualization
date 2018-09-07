/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Kei Okada nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DYNAMIC_TF_PUBLISHER_H
#define DYNAMIC_TF_PUBLISHER_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_interactive_marker/TransformParameterConfig.h>
#include <geometry_msgs/TransformStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

namespace jsk_interactive_marker {

class DynamicTfPublisher {
  typedef jsk_interactive_marker::TransformParameterConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;

  ros::NodeHandle nh_;
  std::string node_name_;

  tf::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped transformStamped;

  ros::Timer publish_timer_;
  double publish_period_;

  boost::mutex mutex_;

  double x_, y_, z_, roll_, pitch_, yaw_;
  double qx_, qy_, qz_, qw_;

  static bool need_config_update_;

  std::string frame_id_;
  std::string parent_frame_id_;
public:
  virtual void onInit();
  virtual void reconfiguration_callback(Config &config, uint32_t level);
  virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  virtual void timerCallback(const ros::TimerEvent);
  virtual visualization_msgs::InteractiveMarker createMarker();
};

} // end-of-namespace jsk_interactive_marker

#endif
