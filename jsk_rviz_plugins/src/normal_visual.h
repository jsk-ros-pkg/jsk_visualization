// -*- mode:c++ -*-
#ifndef NORMAL_VISUAL_H
#define NORMAL_VISUAL_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

namespace jsk_rviz_plugins
{

  class NormalVisual
  {
  public:
    NormalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

    virtual ~NormalVisual();

    void setValues( float x, float y, float z, float normal_x, float normal_y, float  normal_z);
    void setFramePosition( const Ogre::Vector3& position );
    void setFrameOrientation( const Ogre::Quaternion& orientation );
    void setColor( float r, float g, float b, float a );
    void setScale( float scale );

  private:
#if ROS_VERSION_MINIMUM(1,12,0)
    std::shared_ptr<rviz::Arrow> normal_arrow_;
#else
    boost::shared_ptr<rviz::Arrow> normal_arrow_;
#endif
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
  };
}
#endif
