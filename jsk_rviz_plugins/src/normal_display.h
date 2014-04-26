// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/enum_property.h>
#include "normal_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class Array;
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
  class ColorProperty;
}

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
