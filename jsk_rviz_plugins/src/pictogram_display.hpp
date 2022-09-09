#ifndef JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
//#include <rviz_common/message_filter_display.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <chrono>
#include <rclcpp/duration.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <std_msgs/msg/float32.hpp>

#include "facing_visualizer.hpp"
#include "jsk_rviz_plugin_msgs/msg/pictogram.hpp"
#endif
#include <mutex>

namespace jsk_rviz_plugins
{
void setupFont();
int addFont(unsigned char * data, unsigned int data_len);
bool isFontAwesome(std::string);
bool isEntypo(std::string);
bool isCharacterSupported(std::string character);
QFont getFont(std::string character);
QString lookupPictogramText(std::string character);
////////////////////////////////////////////////////////
// PictogramObject
////////////////////////////////////////////////////////
class PictogramObject : public FacingTexturedObject
{
public:
  typedef std::shared_ptr<PictogramObject> Ptr;
  PictogramObject(Ogre::SceneManager * manager, Ogre::SceneNode * parent, double size);
  void update(float wall_dt, float ros_dt) override;
  //void reset() override;
  void setEnable(bool enable);
  void setText(std::string text);
  void setAlpha(double alpha);
  void setColor(QColor color);
  void setSize(double size);
  void setSpeed(double speed);
  void setPose(const geometry_msgs::msg::Pose & pose, const std::string & frame_id);
  void start();
  void setContext(rviz_common::DisplayContext * context);
  void setAction(uint8_t action);
  void setMode(uint8_t mode);
  void setTTL(double ttl);

protected:
  void updatePose(float dt);
  void updateColor();
  void updateText();

  bool need_to_update_;
  uint8_t action_;
  geometry_msgs::msg::Pose pose_;
  std::string frame_id_;
  rviz_common::DisplayContext * context_;
  //ros::WallTime time_;
  rclcpp::Time time_;
  rclcpp::Clock clock_;
  double ttl_;
  double speed_;
  uint8_t mode_;

private:
};

////////////////////////////////////////////////////////
// Display to visualize pictogram on rviz
////////////////////////////////////////////////////////
class PictogramDisplay : public rviz_common::RosTopicDisplay<jsk_rviz_plugin_msgs::msg::Pictogram>
{
  Q_OBJECT
public:
  PictogramDisplay();
  ~PictogramDisplay();

protected:
  ////////////////////////////////////////////////////////
  // methods
  ////////////////////////////////////////////////////////
  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void processMessage(jsk_rviz_plugin_msgs::msg::Pictogram::ConstSharedPtr msg);
  void update(float wall_dt, float ros_dt) override;
  //void reset() override;

  ////////////////////////////////////////////////////////
  // parameters
  ////////////////////////////////////////////////////////
  std::mutex mutex_;
  PictogramObject::Ptr pictogram_;
private Q_SLOTS:

private:
};
}  // namespace jsk_rviz_plugins

#endif
