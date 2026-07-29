// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H
#ifndef Q_MOC_RUN
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <QColor>

#include <deque>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include "normal_visual.h"
#endif

namespace jsk_rviz_plugins
{

class NormalDisplay: public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();
  rviz_common::properties::EnumProperty* style_property_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::ColorProperty* min_color_property_;
  rviz_common::properties::ColorProperty* max_color_property_;
  rviz_common::properties::FloatProperty* skip_rate_property_;
  rviz_common::properties::BoolProperty* rainbow_property_;
  rviz_common::properties::FloatProperty* scale_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
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

  std::deque<std::shared_ptr<NormalVisual>> visuals_;
  size_t visuals_capacity_;

  static int32_t findChannelIndex(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
    const std::string& channel)
  {
    for (size_t i = 0; i < cloud->fields.size(); ++i) {
      if (cloud->fields[i].name == channel) {
        return static_cast<int32_t>(i);
      }
    }
    return -1;
  }

private Q_SLOTS:
  void processMessage( sensor_msgs::msg::PointCloud2::ConstSharedPtr msg );
  void updateStyle();
  void updateSkipRate();
  void updateRainbow();
  void updateScale();
  void updateAlpha();
  void getRainbow(float value , float& rf, float& gf, float& bf);
};

}

#endif
