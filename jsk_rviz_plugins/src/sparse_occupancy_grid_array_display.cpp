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

#include "sparse_occupancy_grid_array_display.h"

namespace jsk_rviz_plugin
{
  SparseOccupancyGridArrayDisplay::SparseOccupancyGridArrayDisplay()
  {
    alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the polygon.",
      this, SLOT( updateAlpha() ));

    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    axis_color_property_ = new rviz::BoolProperty("Axis Color", false,
                                                  "coloring according to the angle of the plane",
                                                  this, SLOT(updateAxisColor()));
    max_color_property_ = new rviz::ColorProperty("Max Color", QColor(255, 255, 255),
                                                  "maximum color to draw grid map",
                                                  this, SLOT(updateMaxColor()));
    min_color_property_ = new rviz::ColorProperty("Min Color", QColor(0, 0, 0),
                                                  "minimum color to draw grid map",
                                                  this, SLOT(updateMinColor()));
  }

  SparseOccupancyGridArrayDisplay::~SparseOccupancyGridArrayDisplay()
  {
    delete alpha_property_;
    delete max_color_property_;
    delete min_color_property_;
    delete axis_color_property_;
    allocateCloudsAndNodes(0);
  }

  void SparseOccupancyGridArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    updateAlpha();
    updateMaxColor();
    updateMinColor();
  }
  
  void SparseOccupancyGridArrayDisplay::allocateCloudsAndNodes(const size_t num)
  {
    if (num > clouds_.size()) { // need to allocate new node and clouds
      for (size_t i = clouds_.size(); i < num; i++) {
        Ogre::SceneNode* node = scene_node_->createChildSceneNode();
        rviz::PointCloud* cloud = new rviz::PointCloud();
        cloud->setRenderMode(rviz::PointCloud::RM_TILES);
        cloud->setCommonDirection( Ogre::Vector3::UNIT_Z );
        cloud->setCommonUpVector( Ogre::Vector3::UNIT_Y );
        node->attachObject(cloud);
        clouds_.push_back(cloud);
        nodes_.push_back(node);
      }
    }
    else if (num < clouds_.size()) // need to destroy
    {
      for (size_t i = num; i < clouds_.size(); i++) {
        nodes_[i]->detachObject(clouds_[i]);
        delete clouds_[i];
        scene_manager_->destroySceneNode(nodes_[i]);
      }
      clouds_.resize(num);
      nodes_.resize(num);
    }
  }

  void SparseOccupancyGridArrayDisplay::reset()
  {
    MFDClass::reset();
    allocateCloudsAndNodes(0);
  }

  QColor SparseOccupancyGridArrayDisplay::gridColor(double value)
  {
    // normalize to 0~1
    return QColor(
      (value * (max_color_.red() - min_color_.red()) + min_color_.red()),
      (value * (max_color_.green() - min_color_.green()) + min_color_.green()),
      (value * (max_color_.blue() - min_color_.blue()) + min_color_.blue()));
  }

  QColor SparseOccupancyGridArrayDisplay::axisColor(const Ogre::Quaternion& q,
                                                    const Ogre::Vector3& p)
  {
    Ogre::Vector3 zaxis = q.zAxis();
    Ogre::Vector3 reference = p.normalisedCopy();
    double dot = zaxis.dotProduct(reference);
    if (dot < -1) {
      dot = -1.0;
    }
    else if (dot > 1) {
      dot = 1.0;
    }
    double scale = (dot + 1) / 2.0;
    return gridColor(scale);
  }
  
  void SparseOccupancyGridArrayDisplay::processMessage(
    const jsk_pcl_ros::SparseOccupancyGridArray::ConstPtr& msg)
  {
    allocateCloudsAndNodes(msg->grids.size()); // not enough
    for (size_t i = 0; i < msg->grids.size(); i++) {
      Ogre::SceneNode* node = nodes_[i];
      rviz::PointCloud* cloud = clouds_[i];
      const jsk_pcl_ros::SparseOccupancyGrid grid = msg->grids[i];
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      if(!context_->getFrameManager()->transform(grid.header, grid.origin_pose,
                                                 position,
                                                 quaternion)) {
        ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), grid.header.frame_id.c_str(),
                   qPrintable( fixed_frame_ ));
        return;                 // return?
      }
      node->setPosition(position);
      node->setOrientation(quaternion);
      cloud->setDimensions(grid.resolution, grid.resolution, 0.0);
      std::vector<rviz::PointCloud::Point> points;
      for (size_t ci = 0; ci < grid.columns.size(); ci++) {
        const jsk_pcl_ros::SparseOccupancyGridColumn column = grid.columns[ci];
        const int column_index = column.column_index;
        for (size_t ri = 0; ri < column.cells.size(); ri++) {
          const jsk_pcl_ros::SparseOccupancyGridCell cell = column.cells[ri];
          const int row_index = cell.row_index;
          rviz::PointCloud::Point point;
          if (!axis_color_) {
            QColor color = gridColor(cell.value);
            Ogre::ColourValue ogre_color = rviz::qtToOgre(color);
            point.color = ogre_color;
          }
          else {
            QColor color = axisColor(quaternion, Ogre::Vector3(1, 0, 0));
            Ogre::ColourValue ogre_color = rviz::qtToOgre(color);
            point.color = ogre_color;
          }
          point.position.x = grid.resolution * (column_index + 0.5);
          point.position.y = grid.resolution * (row_index + 0.5);
          point.position.z = 0.0;
          
          points.push_back(point);
        }
      }
      cloud->clear();
      cloud->setAlpha(alpha_);
      if (!points.empty()) {
        cloud->addPoints(&points.front(), points.size());
      }
    }
    context_->queueRender();
  }
  
  void SparseOccupancyGridArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void SparseOccupancyGridArrayDisplay::updateMaxColor()
  {
    max_color_ = max_color_property_->getColor();
  }

  void SparseOccupancyGridArrayDisplay::updateMinColor()
  {
    min_color_ = min_color_property_->getColor();
  }

  void SparseOccupancyGridArrayDisplay::updateAxisColor()
  {
    axis_color_ = axis_color_property_->getBool();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::SparseOccupancyGridArrayDisplay, rviz::Display )

