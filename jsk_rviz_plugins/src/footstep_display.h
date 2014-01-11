// -*- mode: c++; -*-
#ifndef FOOTSTEP_DISPLAY_H
#define FOOTSTEP_DISPLAY_H

#include <jsk_footstep_msgs/FootstepArray.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#include <OGRE/OgreSceneNode.h>

namespace jsk_rviz_plugin
{
  class FootstepDisplay : public rviz::MessageFilterDisplay<jsk_footstep_msgs::FootstepArray>
  {
    Q_OBJECT
  public:
    FootstepDisplay();
    virtual ~FootstepDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
  private Q_SLOTS:
    void updateAlpha();
    void updateFootstepSize();
  private:
    void processMessage(const jsk_footstep_msgs::FootstepArray::ConstPtr& msg);
    bool validateFloats( const jsk_footstep_msgs::FootstepArray& msg );
    rviz::FloatProperty *alpha_property_;
    rviz::FloatProperty *width_property_;
    rviz::FloatProperty *height_property_;
    rviz::FloatProperty *depth_property_;
    jsk_footstep_msgs::FootstepArray::ConstPtr latest_footstep_;
    typedef boost::shared_ptr<rviz::Shape> ShapePtr;
    std::vector<ShapePtr> shapes_;
    rviz::BillboardLine* line_;
    //Ogre::SceneNode* scene_node_;
  };
}

#endif
