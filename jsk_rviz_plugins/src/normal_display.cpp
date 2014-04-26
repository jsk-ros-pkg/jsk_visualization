// -*- mode: C++ -*-
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>

#include "normal_display.h"

using namespace rviz;

namespace jsk_rviz_plugin
{

NormalDisplay::NormalDisplay()
{
}

void NormalDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

NormalDisplay::~NormalDisplay()
{
}

void NormalDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}


void NormalDisplay::processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg )
{
  //check x,y,z
  int32_t xi = findChannelIndex(msg, "x");
  int32_t yi = findChannelIndex(msg, "y");
  int32_t zi = findChannelIndex(msg, "z");

  if (xi == -1 || yi == -1 || zi == -1)
  {
    return;
  }

  const uint32_t xoff = msg->fields[xi].offset;
  const uint32_t yoff = msg->fields[yi].offset;
  const uint32_t zoff = msg->fields[zi].offset;

  //check normals x,y,z
  int32_t normal_xi = findChannelIndex(msg, "normal_x");
  int32_t normal_yi = findChannelIndex(msg, "normal_y");
  int32_t normal_zi = findChannelIndex(msg, "normal_z");

  if (normal_xi == -1 || normal_yi == -1 || normal_zi == -1)
  {
    return;
  }

  const uint32_t normal_xoff = msg->fields[normal_xi].offset;
  const uint32_t normal_yoff = msg->fields[normal_yi].offset;
  const uint32_t normal_zoff = msg->fields[normal_zi].offset;


  //check other option values
  const uint32_t point_step = msg->point_step;
  const size_t point_count = msg->width * msg->height;

  if (point_count == 0)
  {
    return;
  }


  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  visuals_.clear();
  const uint8_t* ptr = &msg->data.front();
  for (size_t i = 0; i < point_count; ++i)
  {
    float x = *reinterpret_cast<const float*>(ptr + xoff);
    float y = *reinterpret_cast<const float*>(ptr + yoff);
    float z = *reinterpret_cast<const float*>(ptr + zoff);
    float normal_x = *reinterpret_cast<const float*>(ptr + normal_xoff);
    float normal_y = *reinterpret_cast<const float*>(ptr + normal_yoff);
    float normal_z = *reinterpret_cast<const float*>(ptr + normal_zoff);
    if (validateFloats(Ogre::Vector3(x, y, z)) && validateFloats(Ogre::Vector3(normal_x, normal_y, normal_z)))
    {
      boost::shared_ptr<NormalVisual> visual(new NormalVisual( context_->getSceneManager(), scene_node_ ));
      visual->setValues( x, y, z, normal_x, normal_y, normal_z );
      visual->setFramePosition( position );
      visual->setFrameOrientation( orientation );
      visual->setColor( 0, 0, 0, 0 );
      visuals_.push_back(visual);
    }

    ptr += point_step;
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::NormalDisplay,rviz::Display )
