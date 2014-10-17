#ifndef TRANSFORMABLE_MARKER_OPERATOR_H
#define TRANSFORMABLE_MARKER_OPERATOR_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include <QtGui>
#include <jsk_interactive_marker/RequestMarkerOperate.h>

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;
class PropertyTreeWidget;


namespace jsk_rviz_plugin
{
  class TransformableMarkerOperatorAction: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      TransformableMarkerOperatorAction( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      protected Q_SLOTS:

      void callRequestMarkerOperateService(jsk_interactive_marker::RequestMarkerOperate srv);
      void insertBoxService();
      void insertCylinderService();
      void insertTorusService();
      void eraseWithIdService();
      void eraseAllService();
      void eraseFocusService();

    protected:
      QPushButton* insert_box_button_;
      QPushButton* insert_cylinder_button_;
      QPushButton* insert_torus_button_;

      QPushButton* erase_with_id_button_;
      QPushButton* erase_all_button_;
      QPushButton* erase_focus_button_;

      QVBoxLayout* layout;

      QLineEdit* name_editor_;
      QLineEdit* description_editor_;
      QLineEdit* frame_editor_;
      QLineEdit* id_editor_;

      // The ROS node handle.
      ros::NodeHandle nh_;

    };

}

#endif // TELEOP_PANEL_H
