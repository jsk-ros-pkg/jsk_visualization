// -*- mode:c++ -*-
#ifndef NORMAL_VISUAL_H
#define NORMAL_VISUAL_H

#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_rendering/objects/arrow.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <ros/ros.hpp>

namespace jsk_rviz_plugins
{

  class NormalVisual
  {
  public:
    NormalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

    ~NormalVisual();

    void setValues( float x, float y, float z, float normal_x, float normal_y, float  normal_z);
    void setFramePosition( const Ogre::Vector3& position );
    void setFrameOrientation( const Ogre::Quaternion& orientation );
    void setColor( float r, float g, float b, float a );
    void setScale( float scale );

  private:
    std::shared_ptr<rviz_rendering::Arrow> normal_arrow_;

    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
  };
}
#endif