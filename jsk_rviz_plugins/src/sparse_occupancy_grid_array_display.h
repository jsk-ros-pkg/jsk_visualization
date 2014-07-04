// -*- mode: c++; -*-
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

#ifndef JSK_RVIZ_PLUGINS_SPARSE_OCCUPANCY_GRID_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_SPARSE_OCCUPANCY_GRID_ARRAY_DISPLAY_H_

#include <jsk_pcl_ros/SparseOccupancyGridArray.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/ogre_helpers/point_cloud.h>

namespace jsk_rviz_plugin
{
  class SparseOccupancyGridArrayDisplay: public rviz::MessageFilterDisplay<jsk_pcl_ros::SparseOccupancyGridArray>
  {
    Q_OBJECT
  public:
    typedef boost::shared_ptr<rviz::PointCloud> PointCloudPtr;
    SparseOccupancyGridArrayDisplay();
    virtual ~SparseOccupancyGridArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void allocateCloudsAndNodes(const size_t num);
    virtual QColor gridColor(double value);
    // ?? do we need to track the planes for efficient rendering?
    rviz::ColorProperty* max_color_property_;
    rviz::ColorProperty* min_color_property_;
    rviz::FloatProperty* alpha_property_;
    double alpha_;
    std::vector<rviz::PointCloud*> clouds_;
    std::vector<Ogre::SceneNode*> nodes_;
    QColor max_color_;
    QColor min_color_;
  private:
    void processMessage(
      const jsk_pcl_ros::SparseOccupancyGridArray::ConstPtr& msg);
  private Q_SLOTS:
    void updateAlpha();
    void updateMaxColor();
    void updateMinColor();
  };

}

#endif
