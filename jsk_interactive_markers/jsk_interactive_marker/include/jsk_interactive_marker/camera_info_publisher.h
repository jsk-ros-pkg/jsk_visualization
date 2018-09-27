// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include "jsk_interactive_marker/interactive_marker_helpers.h"
#include <dynamic_reconfigure/server.h>
#include "jsk_interactive_marker/CameraInfoPublisherConfig.h"

namespace jsk_interactive_marker
{
  class CameraInfoPublisher
  {
  public:
    typedef std::shared_ptr<CameraInfoPublisher> Ptr;
    typedef jsk_interactive_marker::CameraInfoPublisherConfig Config;
    CameraInfoPublisher();
    virtual ~CameraInfoPublisher();
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void publishCameraInfo(const ros::Time& stamp);
    virtual void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    virtual void initializeInteractiveMarker();
    virtual void pointcloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void imageCallback(
      const sensor_msgs::Image::ConstPtr& msg);
    virtual void staticRateCallback(
      const ros::TimerEvent& event);
    virtual void configCallback(Config &config, uint32_t level);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Publisher pub_camera_info_;
    ros::Subscriber sub_sync_;
    ros::Timer timer_;
    std::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    ////////////////////////////////////////////////////////
    // variables
    ////////////////////////////////////////////////////////
    std::string frame_id_;
    std::string parent_frame_id_;
    double width_;
    double height_;
    double f_;
    geometry_msgs::Pose latest_pose_;
    
  private:
    
  };
}
