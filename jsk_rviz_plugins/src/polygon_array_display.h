// -*- mode: c++; -*-

#ifndef POLYGON_ARRAY_DISPLAY_H
#define POLYGON_ARRAY_DISPLAY_H

#include <jsk_pcl_ros/PolygonArray.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace jsk_rviz_plugin
{
  class PolygonArrayDisplay : public rviz::MessageFilterDisplay<jsk_pcl_ros::PolygonArray>
  {
    Q_OBJECT
  public:
    PolygonArrayDisplay();
    virtual ~PolygonArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void updateSceneNodes(const jsk_pcl_ros::PolygonArray::ConstPtr& msg);
    virtual void allocateMaterials(int num);
    virtual void updateLines(int num);
  private Q_SLOTS:
    void updateAutoColoring();
    void updateOnlyBorder();
  private:
    void processMessage(const jsk_pcl_ros::PolygonArray::ConstPtr& msg);
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* only_border_property_;
    rviz::BoolProperty* auto_coloring_property_;
    bool only_border_;
    bool auto_coloring_;
    std::vector<Ogre::ManualObject*> manual_objects_;
    std::vector<Ogre::SceneNode*> scene_nodes_;
    std::vector<Ogre::MaterialPtr> materials_;
    std::vector<rviz::BillboardLine*> lines_;
  };
}

#endif

