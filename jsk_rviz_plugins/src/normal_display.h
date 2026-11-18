// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H
// outside of Q_MOC_RUN: moc reads ROS_VERSION_MAJOR for processMessage() below
#include "rviz_compat.h"
#ifndef Q_MOC_RUN
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <QColor>

#include <boost/circular_buffer.hpp>

#if ROS_VERSION_MAJOR != 1
#include <sensor_msgs/msg/point_cloud2.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, PointCloud2)

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#else
#include <sensor_msgs/PointCloud2.h>

#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#endif
#include "normal_visual.h"
#endif

namespace jsk_rviz_plugins
{

class NormalDisplay: public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();
  rviz::EnumProperty* style_property_;
  rviz::ColorProperty* color_property_;
  rviz::ColorProperty* min_color_property_;
  rviz::ColorProperty* max_color_property_;
  rviz::FloatProperty* skip_rate_property_;
  rviz::BoolProperty* rainbow_property_;
  rviz::FloatProperty* scale_property_;
  rviz::FloatProperty* alpha_property_;
  float skip_rate_;
  float scale_;
  float alpha_;

  enum ColorTypes{
    POINTS_COLOR,
    FLAT_COLOR,
    DIRECTION_COLOR,
    CURVATURE_COLOR
  };

protected:
  virtual void onInitialize();

  virtual void reset();

#if ROS_VERSION_MINIMUM(1,12,0)
  boost::circular_buffer<std::shared_ptr<NormalVisual> > visuals_;
#else
  boost::circular_buffer<boost::shared_ptr<NormalVisual> > visuals_;
#endif


  // Function to handle an incoming ROS message.
private Q_SLOTS:
#if ROS_VERSION_MAJOR != 1
  void processMessage( sensor_msgs::PointCloud2::ConstSharedPtr msg );
#else
  void processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg );
#endif
  void updateStyle();
  void updateSkipRate();
  void updateRainbow();
  void updateScale();
  void updateAlpha();
  void getRainbow(float value , float& rf, float& gf, float& bf);
};

}

#endif
