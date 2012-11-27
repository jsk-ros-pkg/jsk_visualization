#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>

#include <ros/ros.h>

#include "wrench_visual.h"

namespace jsk_rviz_plugin
{

    WrenchStampedVisual::WrenchStampedVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    {
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the WrenchStamped's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	arrow_force_ = new rviz::Arrow( scene_manager_, frame_node_ );
	arrow_torque_ = new rviz::Arrow( scene_manager_, frame_node_ );
    }

    WrenchStampedVisual::~WrenchStampedVisual()
    {
	// Delete the arrow to make it disappear.
	//delete acceleration_arrow_;

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
    }


    void WrenchStampedVisual::setMessage( const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
        Ogre::Vector3 force(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        Ogre::Vector3 torque(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
        double force_length = force.length() * 0.001 * scale_;
        double torque_length = torque.length() * 0.001 * scale_;
	arrow_force_->setScale(Ogre::Vector3(width_, width_, force_length)); 
	arrow_torque_->setScale(Ogre::Vector3(width_, width_, torque_length));

        arrow_force_->setDirection(force);
        arrow_torque_->setDirection(torque);
    }

    // Position and orientation are passed through to the SceneNode.
    void WrenchStampedVisual::setFramePosition( const Ogre::Vector3& position )
    {
	frame_node_->setPosition( position );
    }

    void WrenchStampedVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
	frame_node_->setOrientation( orientation );
    }

    // Color is passed through to the rviz object.
    void WrenchStampedVisual::setForceColor( float r, float g, float b, float a )
    {
	arrow_force_->setColor( r, g, b, a );
    }
    // Color is passed through to the rviz object.
    void WrenchStampedVisual::setTorqueColor( float r, float g, float b, float a )
    {
	arrow_torque_->setColor( r, g, b, a );
    }

    void  WrenchStampedVisual::setScale( float s ) {
      scale_ = s;
    }
    void  WrenchStampedVisual::setWidth( float w ) {
      width_ = w;
    }

} // end namespace jsk_rviz_plugin

