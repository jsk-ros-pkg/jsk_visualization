// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <rviz/message_filter_display.h>
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
}

namespace jsk_rviz_plugin
{

class NormalDisplay: public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();


  // Function to handle an incoming ROS message.
private:
  void processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg );

  boost::circular_buffer<boost::shared_ptr<NormalVisual> > visuals_;

};

}

#endif
