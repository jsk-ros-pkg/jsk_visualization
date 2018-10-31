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


#ifndef JSK_INTERACTIVE_MARKER_POINTCLOUD_CLOPPER_H_
#define JSK_INTERACTIVE_MARKER_POINTCLOUD_CLOPPER_H_

#define BOOST_PARAMETER_MAX_ARITY 7 // its a hack

#include <jsk_topic_tools/time_accumulator.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_interactive_marker/PointCloudCropperConfig.h>

namespace jsk_interactive_marker
{
  
  class Cropper
  {
  public:
    typedef std::shared_ptr<Cropper> Ptr;
    Cropper(const unsigned int nr_parameter);
    virtual ~Cropper();
    
    virtual void crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    virtual std::string getName() = 0;
    virtual visualization_msgs::Marker getMarker() = 0;
    virtual void updateParameter(const double val, const unsigned int index);
    // return true if the point p is inside of the cropper
    virtual bool isInside(const pcl::PointXYZ& p) = 0;
    virtual void fillInitialParameters() = 0;
    virtual void setPose(Eigen::Affine3f pose);
    virtual Eigen::Affine3f getPose();
  protected:
    unsigned int nr_parameter_;
    std::vector<double> parameters_;
    // pose_ of the Cropper should be respected to the frame_id of pointcloud
    Eigen::Affine3f pose_;
  private:
    
  };

  class SphereCropper: public Cropper
  {
  public:
    typedef std::shared_ptr<SphereCropper> Ptr;
    
    SphereCropper();
    virtual ~SphereCropper();
    virtual std::string getName();
    virtual visualization_msgs::Marker getMarker();
    virtual bool isInside(const pcl::PointXYZ& p);
    virtual void fillInitialParameters();
    virtual double getRadius();
  protected:
  private:
    
  };

  class CubeCropper: public Cropper
  {
  public:
    typedef std::shared_ptr<CubeCropper> Ptr;
    
    CubeCropper();
    virtual ~CubeCropper();
    virtual std::string getName();
    virtual visualization_msgs::Marker getMarker();
    virtual bool isInside(const pcl::PointXYZ& p);
    virtual void fillInitialParameters();
    virtual double getWidthX();
    virtual double getWidthY();
    virtual double getWidthZ();
  protected:
  private:
    
  };

  
  class PointCloudCropper
  {
  public:
    PointCloudCropper(ros::NodeHandle& nh, ros::NodeHandle &pnh);
    virtual ~PointCloudCropper();
  protected:
    typedef PointCloudCropperConfig Config;
    
    typedef std::vector<interactive_markers::MenuHandler::EntryHandle>
    EntryHandleVector;
    virtual void changeCropper(Cropper::Ptr next_cropper);
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void reInitializeInteractiveMarker();
    virtual void updateInteractiveMarker(
      Eigen::Affine3f pose_offset = Eigen::Affine3f::Identity());
    virtual void initializeInteractiveMarker(
      Eigen::Affine3f pose_offset = Eigen::Affine3f::Identity());
    virtual void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    virtual void menuFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    virtual void changeCropperCallback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    virtual void updateMenuCheckboxStatus();
    virtual void cropAndPublish(ros::Publisher& pub);
    virtual void configCallback(Config &config, uint32_t level);
    boost::mutex mutex_;
    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;
    ros::Publisher point_visualization_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr latest_pointcloud_;
    sensor_msgs::PointCloud2::ConstPtr latest_pointcloud_;
    std::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    interactive_markers::MenuHandler menu_handler_;
    Cropper::Ptr cropper_;
    std::vector<Cropper::Ptr> cropper_candidates_;
    EntryHandleVector cropper_entries_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    
  private:
  };
}

#endif
