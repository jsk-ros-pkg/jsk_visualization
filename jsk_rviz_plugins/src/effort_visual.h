#ifndef EFFORT_VISUAL_H
#define EFFORT_VISUAL_H

#include <sensor_msgs/JointState.h>

namespace Ogre
{
    class Vector3;
    class Quaternion;
}

namespace rviz
{
    class BillboardLine;
}

namespace jsk_rviz_plugin
{


// Each instance of EffortVisual represents the visualization of a single
// sensor_msgs::Effort message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class EffortVisual
{
public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    EffortVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~EffortVisual();

    // set rainbow color
    void getRainbowColor(float value, Ogre::ColourValue& color);
    // Configure the visual to show the data in the message.
    void setMessage( const double effort_scale );

    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean.  This way EffortVisual is only
    // responsible for visualization.
    void setFramePosition( const Ogre::Vector3& position );
    void setFrameOrientation( const Ogre::Quaternion& orientation );

    // Set the color and alpha of the visual, which are user-editable
    // parameters and therefore don't come from the Effort message.
    void setColor( float r, float g, float b, float a );

    void setWidth( float w );

private:
    // The object implementing the effort circle
    rviz::BillboardLine* effort_circle_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the Effort message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;

    float width_;

};

} // end namespace jsk_rviz_plugin

#endif // EFFORT_VISUAL_H
