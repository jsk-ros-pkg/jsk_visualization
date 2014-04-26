// -*- mode:c++ -*-

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/Vector3.h>
#include "normal_visual.h"

namespace jsk_rviz_plugin
{

NormalVisual::NormalVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  acceleration_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ ));
}

NormalVisual::~NormalVisual()
{
  scene_manager_->destroySceneNode( frame_node_ );
}

  void NormalVisual::setValues( float x, float y, float z, float normal_x, float normal_y, float  normal_z)
{

  Ogre::Vector3 dir( normal_x, normal_y, normal_z);
  Ogre::Vector3 pos(        x,        y,        z);

  float length = dir.length()/10;

  Ogre::Vector3 scale( length, length, length );
  // Ogre::Vector3 scale( 10, 10, 10 );
  acceleration_arrow_->setScale( scale );
  acceleration_arrow_->setDirection( dir );
  acceleration_arrow_->setPosition(pos);
}

void NormalVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void NormalVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

void NormalVisual::setColor( float r, float g, float b, float a )
{
  acceleration_arrow_->setColor( r, g, b, a );
}

} // end namespace jsk_rviz_plugin

