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

#include "pictogram_display.h"
#include <QPainter>
#include <QFontDatabase>
#include <ros/package.h>

namespace jsk_rviz_plugin
{

  PictogramObject::PictogramObject(Ogre::SceneManager* manager,
                                   Ogre::SceneNode* parent,
                                   double size):
    FacingTexturedObject(manager, parent, size)
  {
    square_object_->setPolygonType(SquareObject::SQUARE);
    square_object_->rebuildPolygon();
  }
  
  void PictogramObject::update(float wall_dt, float ros_dt)
  {
    ScopedPixelBuffer buffer = texture_object_->getBuffer();
    QColor transparent(255, 255, 255, 0);
    QColor white(25, 255, 240, 255);
    QImage Hud = buffer.getQImage(128, 128, transparent); // should change according to size
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);
    QColor foreground = rviz::ogreToQt(color_);
    painter.setPen(QPen(foreground, 5, Qt::SolidLine));
    painter.setBrush(white);
    //QFont font("DejaVu Sans Mono");
    QFont font("Entypo");
    font.setPointSize(200);
    font.setBold(true);
    painter.setFont(font);
    //const QChar data[1] = {0x2753};
    const QChar data[1] = {0x1F3AC};
    painter.drawText(0, 128, QString(data, 1));
    painter.end();
  }

  void PictogramObject::updateColor()
  {
  }
  void PictogramObject::updateText()
  {
  }
  
  PictogramDisplay::PictogramDisplay()
  {
    // register font
    // std::string font_path
    //   = resource_retriever::Retriever().get("package://jsk_rviz_plugins/resources/fonts/Entypo.ttf");
    ROS_INFO("%s", ros::package::getPath("jsk_rviz_plugins").c_str());
    QFontDatabase::addApplicationFont((ros::package::getPath("jsk_rviz_plugins") + "/resources/fonts/Entypo.ttf").c_str());
    QFontDatabase::addApplicationFont((ros::package::getPath("jsk_rviz_plugins") + "/resources/fonts/Entypo-Social.ttf").c_str());
  }

  PictogramDisplay::~PictogramDisplay()
  {
    
  }
  
  void PictogramDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    pictogram_.reset(new PictogramObject(scene_manager_,
                                         scene_node_,
                                         1.0));
    pictogram_->setEnable(false);
    // initial setting
    pictogram_->setColor(QColor(25, 255, 240));
    pictogram_->setAlpha(1.0);
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  void PictogramDisplay::reset()
  {
    MFDClass::reset();
  }

  void PictogramDisplay::onEnable()
  {
    subscribe();
    if (pictogram_) {
      // keep false, it will be true
      // in side of processMessae callback.
      pictogram_->setEnable(false);
    }
  }

  void PictogramDisplay::processMessage(const jsk_rviz_plugins::Pictogram::ConstPtr& msg)
  {
    boost::mutex::scoped_lock (mutex_);
    pictogram_->setEnable(isEnabled());
    if (!isEnabled()) {
      return;
    }
    Ogre::Vector3 position;
    Ogre::Quaternion quaternion;
    if(!context_->getFrameManager()->transform(msg->header,
                                               msg->pose,
                                               position,
                                               quaternion)) {
      ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                 qPrintable( getName() ), msg->header.frame_id.c_str(),
                 qPrintable( fixed_frame_ ));
      return;
    }
    pictogram_->setPosition(position);
    pictogram_->setOrientation(quaternion);
  }

  void PictogramDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock (mutex_);
    if (pictogram_) {
      pictogram_->update(wall_dt, ros_dt);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugin::PictogramDisplay, rviz::Display);
