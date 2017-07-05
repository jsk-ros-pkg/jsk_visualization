#ifndef TRANSFORMABLE_MARKER_OPERATOR_H
#define TRANSFORMABLE_MARKER_OPERATOR_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <jsk_interactive_marker/SetMarkerDimensions.h>
#include <jsk_recognition_msgs/ObjectArray.h>
#include <jsk_rviz_plugins/RequestMarkerOperate.h>
#endif

class QLineEdit;
class QPushButton;

namespace rviz {
  class VisualizationManager;
}

namespace jsk_interactive_marker
{
  class TransformableMarkerOperatorAction: public rviz::Panel
    {
      Q_OBJECT
      public:
      TransformableMarkerOperatorAction( QWidget* parent = 0 );

      virtual void onInitialize();
      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

    protected Q_SLOTS:
      void update();
      void updateServerName();
      void updateObjectArrayTopic();
      void updateFocusMarkerDimensions();
      void updateDimensionsService();
      void updateFrameId();
      void updateName();

      void callRequestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate srv);
      void insertBoxService();
      void insertCylinderService();
      void insertTorusService();
      void insertMeshService();
      void eraseWithIdService();
      void eraseAllService();
      void eraseFocusService();
      void objectArrayCb(const jsk_recognition_msgs::ObjectArray::ConstPtr& obj_array_msg);

    protected:
      QPushButton* insert_box_button_;
      QPushButton* insert_cylinder_button_;
      QPushButton* insert_torus_button_;
      QPushButton* insert_mesh_button_;

      QPushButton* erase_with_id_button_;
      QPushButton* erase_all_button_;
      QPushButton* erase_focus_button_;

      QVBoxLayout* layout;

      QLineEdit* server_name_editor_;
      QLineEdit* topic_name_editor_;
      QLineEdit* transform_name_editor_;
      QLineEdit* dimension_x_editor_;
      QLineEdit* dimension_y_editor_;
      QLineEdit* dimension_z_editor_;
      QComboBox* object_editor_;
      QLineEdit* dimension_radius_editor_;
      QLineEdit* dimension_sm_radius_editor_;
      QLineEdit* name_editor_;
      QLineEdit* description_editor_;
      QLineEdit* frame_editor_;
      QLineEdit* id_editor_;

      std::vector<jsk_recognition_msgs::Object> objects_;

      ros::NodeHandle nh_;
      ros::Subscriber sub_obj_array_;
    };
}  // namespace jsk_interactive_marker

#endif
