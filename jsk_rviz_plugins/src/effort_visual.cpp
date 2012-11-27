#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include "effort_visual.h"

namespace jsk_rviz_plugin
{

    EffortVisual::EffortVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    {
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the Effort's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	effort_circle_ = new rviz::BillboardLine( scene_manager_, frame_node_ );
    }

    EffortVisual::~EffortVisual()
    {
	// Delete the arrow to make it disappear.
	//delete acceleration_arrow_;

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
    }


    void EffortVisual::getRainbowColor(float value, Ogre::ColourValue& color)
    {
	value = std::min(value, 1.0f);
	value = std::max(value, 0.0f);

	float h = value * 5.0f + 1.0f;
	int i = floor(h);
	float f = h - i;
	if ( !(i&1) ) f = 1 - f; // if i is even
	float n = 1 - f;

	if      (i <= 1) color[0] = 0, color[1] = n, color[2] = 1;
	else if (i == 2) color[0] = 0, color[1] = 1, color[2] = n;
	else if (i == 3) color[0] = n, color[1] = 1, color[2] = 0;
	else if (i == 4) color[0] = 1, color[1] = n, color[2] = 0;
	else if (i >= 5) color[0] = n, color[1] = 0, color[2] = 1;
    }

    void EffortVisual::setMessage( const double effort_scale )
    {
	effort_circle_->clear();
	effort_circle_->setLineWidth(width_);
	for (int i = 0; i < 32; i++) {
	    effort_circle_->addPoint(Ogre::Vector3((0.05+effort_scale*0.5)*sin(i*2*M_PI/32), (0.05+effort_scale*0.5)*cos(i*2*M_PI/32), 0));
	}
	Ogre::ColourValue color;
	getRainbowColor(effort_scale, color);
	effort_circle_->setColor(color.r, color.g, color.b, color.a);
    }

    // Position and orientation are passed through to the SceneNode.
    void EffortVisual::setFramePosition( const Ogre::Vector3& position )
    {
	frame_node_->setPosition( position );
    }

    void EffortVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
	frame_node_->setOrientation( orientation );
    }

    // Color is passed through to the rviz object.
    void EffortVisual::setColor( float r, float g, float b, float a )
    {
	effort_circle_->setColor( r, g, b, a );
    }

    void EffortVisual::setWidth( float w )
    {
        width_ = w;
    }
} // end namespace jsk_rviz_plugin

