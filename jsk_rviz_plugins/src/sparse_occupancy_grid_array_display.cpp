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
  const double texture_margin = 1.2;
  
  SparseOccupancyGridArrayDisplay::SparseOccupancyGridArrayDisplay()
  {
    alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the polygon.",
      this, SLOT( updateAlpha() ));

    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

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
  }

  void SparseOccupancyGridArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    updateAlpha();
    updateMaxColor();
    updateMinColor();
  }
  
  void SparseOccupancyGridArrayDisplay::allocateShapes(
    const jsk_pcl_ros::SparseOccupancyGridArray::ConstPtr& msg)
  {
    // countup all the cells
    int num = 0;
    for (size_t i = 0; i < msg->grids.size(); i++) {
      const jsk_pcl_ros::SparseOccupancyGrid grid = msg->grids[i];
      for (size_t ci = 0; ci < grid.columns.size(); ci++) {
        const jsk_pcl_ros::SparseOccupancyGridColumn column = grid.columns[ci];
        for (size_t ri = 0; ri < column.cells.size(); ri++) {
          const jsk_pcl_ros::SparseOccupancyGridCell cell = column.cells[ri];
          ++num;
        }
      }
    }
    if (num > shapes_.size()) {
      for (size_t i = shapes_.size(); i < num; i++) {
        ShapePtr shape (new rviz::Shape(rviz::Shape::Cube, context_->getSceneManager(),
                                        scene_node_));
        shapes_.push_back(shape);
      }
    }
    else if (num < shapes_.size())
    {
      shapes_.resize(num);
    }
  }

  void SparseOccupancyGridArrayDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
  }

  QColor SparseOccupancyGridArrayDisplay::gridColor(double value)
  {
    // normalize to 0~1
    return QColor(
      (value * (max_color_.red() - min_color_.red()) + min_color_.red()) / 255.0,
      (value * (max_color_.green() - min_color_.green()) + min_color_.green()) / 255.0,
      (value * (max_color_.blue() - min_color_.blue()) + min_color_.blue()) / 255.0);
  }
  
  void SparseOccupancyGridArrayDisplay::processMessage(
    const jsk_pcl_ros::SparseOccupancyGridArray::ConstPtr& msg)
  {
    allocateShapes(msg); // not enough
    int cell_counter = 0;
    for (size_t i = 0; i < msg->grids.size(); i++) {
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
      for (size_t ci = 0; ci < grid.columns.size(); ci++) {
        const jsk_pcl_ros::SparseOccupancyGridColumn column = grid.columns[ci];
        const int column_index = column.column_index;
        for (size_t ri = 0; ri < column.cells.size(); ri++) {
          const jsk_pcl_ros::SparseOccupancyGridCell cell = column.cells[ri];
          const int row_index = cell.row_index;
          ShapePtr shape = shapes_[cell_counter];
          Ogre::Vector3 offset(grid.resolution * column_index,
                               grid.resolution * row_index,
                               0);
          Ogre::Vector3 cell_position = position + quaternion * offset;
          shape->setPosition(cell_position);
          shape->setOrientation(quaternion);
          QColor color = gridColor(cell.value);
          shape->setColor(color.red(), color.green(), color.blue(), alpha_);
          shape->setScale(Ogre::Vector3(grid.resolution, grid.resolution, 0.01));
          cell_counter++;
        }
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
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::SparseOccupancyGridArrayDisplay, rviz::Display )

