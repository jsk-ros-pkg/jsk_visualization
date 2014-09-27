/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "bounding_box_array_display.h"
#include <jsk_topic_tools/color_utils.h>
namespace jsk_rviz_plugin
{  
  BoundingBoxArrayDisplay::BoundingBoxArrayDisplay()
  {
    color_property_ = new rviz::ColorProperty("color", QColor(25, 255, 0),
                                              "color to draw the bounding boxes",
                                              this, SLOT(updateColor()));
    alpha_property_ = new rviz::FloatProperty("alpha", 0.8,
                                              "alpha value to draw the bounding boxes",
                                              this, SLOT(updateAlpha()));
    only_edge_property_ = new rviz::BoolProperty("only edge", false,
                                                 "show only the edges of the boxes",
                                                 this, SLOT(updateOnlyEdge()));
    line_width_property_ = new rviz::FloatProperty("line width", 0.005,
                                                   "line width of the edges",
                                                   this, SLOT(updateLineWidth()));
    auto_color_property_ = new rviz::BoolProperty("auto color", false,
                                                  "change the color of the boxes automatically",
                                                  this, SLOT(updateAutoColor()));
  }
  
  BoundingBoxArrayDisplay::~BoundingBoxArrayDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete only_edge_property_;
    delete auto_color_property_;
  }

  QColor BoundingBoxArrayDisplay::getColor(size_t index)
  {
    if (auto_color_) {
      std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(index);
      return QColor(ros_color.r * 255.0, ros_color.g * 255.0, ros_color.b * 255.0,
                    ros_color.a * 255.0);
    }
    else {
      return color_;
    }
  }
  
  void BoundingBoxArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    updateColor();
    updateAlpha();
    updateOnlyEdge();
    updateAutoColor();
    updateLineWidth();
  }

  void BoundingBoxArrayDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
  }
  
  void BoundingBoxArrayDisplay::updateColor()
  {
    color_ = color_property_->getColor();
  }

  void BoundingBoxArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void BoundingBoxArrayDisplay::updateOnlyEdge()
  {
    only_edge_ = only_edge_property_->getBool();
  }

  void BoundingBoxArrayDisplay::updateAutoColor()
  {
    auto_color_ = auto_color_property_->getBool();
  }

  void BoundingBoxArrayDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
  }

  void BoundingBoxArrayDisplay::allocateShapes(int num)
  {
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
  
  void BoundingBoxArrayDisplay::allocateBillboardLines(int num)
  {
    if (num > edges_.size()) {
      for (size_t i = edges_.size(); i < num; i++) {
        BillboardLinePtr line(new rviz::BillboardLine(context_->getSceneManager(), scene_node_));
        edges_.push_back(line);
      }
    }
    else if (num < edges_.size())
    {
      edges_.resize(num);       // ok??
    }
  }

  void BoundingBoxArrayDisplay::processMessage(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
  {
    if (!only_edge_) {
      edges_.clear();
      allocateShapes(msg->boxes.size());
      for (size_t i = 0; i < msg->boxes.size(); i++) {
        jsk_pcl_ros::BoundingBox box = msg->boxes[i];
        ShapePtr shape = shapes_[i];
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        if(!context_->getFrameManager()->transform(box.header, box.pose,
                                                   position,
                                                   quaternion))
        {
          ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                     qPrintable( getName() ), box.header.frame_id.c_str(),
                     qPrintable( fixed_frame_ ));
          return;                 // return?
        }
        shape->setPosition(position);
        shape->setOrientation(quaternion);
        Ogre::Vector3 dimensions;
        dimensions[0] = box.dimensions.x;
        dimensions[1] = box.dimensions.y;
        dimensions[2] = box.dimensions.z;
        shape->setScale(dimensions);
        QColor color = getColor(i);
        shape->setColor(color.red() / 255.0,
                        color.green() / 255.0,
                        color.blue() / 255.0,
                        alpha_);
      }
    }
    else {
      shapes_.clear();
      allocateBillboardLines(msg->boxes.size());
      for (size_t i = 0; i < msg->boxes.size(); i++) {
        jsk_pcl_ros::BoundingBox box = msg->boxes[i];
        geometry_msgs::Vector3 dimensions = box.dimensions;
      
        BillboardLinePtr edge = edges_[i];
        edge->clear();
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        if(!context_->getFrameManager()->transform(box.header, box.pose,
                                                   position,
                                                   quaternion))
        {
          ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                     qPrintable( getName() ), box.header.frame_id.c_str(),
                     qPrintable( fixed_frame_ ));
          return;                 // return?
        }
        edge->setPosition(position);
        edge->setOrientation(quaternion);

        edge->setMaxPointsPerLine(2);
        edge->setNumLines(12);
        edge->setLineWidth(line_width_);
        QColor color = getColor(i);
        edge->setColor(color.red() / 255.0,
                       color.green() / 255.0,
                       color.blue() / 255.0,
                       alpha_);


      
        Ogre::Vector3 A, B, C, D, E, F, G, H;
        A[0] = dimensions.x / 2.0;
        A[1] = dimensions.y / 2.0;
        A[2] = dimensions.z / 2.0;
        B[0] = - dimensions.x / 2.0;
        B[1] = dimensions.y / 2.0;
        B[2] = dimensions.z / 2.0;
        C[0] = - dimensions.x / 2.0;
        C[1] = - dimensions.y / 2.0;
        C[2] = dimensions.z / 2.0;
        D[0] = dimensions.x / 2.0;
        D[1] = - dimensions.y / 2.0;
        D[2] = dimensions.z / 2.0;

        E[0] = dimensions.x / 2.0;
        E[1] = dimensions.y / 2.0;
        E[2] = - dimensions.z / 2.0;
        F[0] = - dimensions.x / 2.0;
        F[1] = dimensions.y / 2.0;
        F[2] = - dimensions.z / 2.0;
        G[0] = - dimensions.x / 2.0;
        G[1] = - dimensions.y / 2.0;
        G[2] = - dimensions.z / 2.0;
        H[0] = dimensions.x / 2.0;
        H[1] = - dimensions.y / 2.0;
        H[2] = - dimensions.z / 2.0;
      
        edge->addPoint(A); edge->addPoint(B); edge->newLine();
        edge->addPoint(B); edge->addPoint(C); edge->newLine();
        edge->addPoint(C); edge->addPoint(D); edge->newLine();
        edge->addPoint(D); edge->addPoint(A); edge->newLine();
        edge->addPoint(E); edge->addPoint(F); edge->newLine();
        edge->addPoint(F); edge->addPoint(G); edge->newLine();
        edge->addPoint(G); edge->addPoint(H); edge->newLine();
        edge->addPoint(H); edge->addPoint(E); edge->newLine();
        edge->addPoint(A); edge->addPoint(E); edge->newLine();
        edge->addPoint(B); edge->addPoint(F); edge->newLine();
        edge->addPoint(C); edge->addPoint(G); edge->newLine();
        edge->addPoint(D); edge->addPoint(H);
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::BoundingBoxArrayDisplay, rviz::Display )
