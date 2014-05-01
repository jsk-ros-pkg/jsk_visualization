// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <QColor>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>

#include "normal_visual.h"

namespace jsk_rviz_plugin
{

class NormalDisplay: public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();
  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;

  enum ColorTypes{
    POINTS_COLOR,
    FLAT_COLOR,
    DIRECTION_COLOR
  };

protected:
  virtual void onInitialize();

  virtual void reset();

  boost::circular_buffer<boost::shared_ptr<NormalVisual> > visuals_;

  // Function to handle an incoming ROS message.
private Q_SLOTS:
  void processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg );
  void updateStyle();

};

}

#endif
