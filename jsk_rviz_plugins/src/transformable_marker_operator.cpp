#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QColor>
#include <QFont>

#include "rviz/properties/property_tree_widget.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "rviz/config.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/color_property.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

#include "transformable_marker_operator.h"
#include "ros/time.h"

using namespace rviz;

namespace jsk_rviz_plugin
{

  TransformableMarkerOperatorAction::TransformableMarkerOperatorAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Button to send cancel topic
    insert_box_button_ = new QPushButton("Insert New Box Marker");
    layout->addWidget( insert_box_button_ );

    insert_cylinder_button_ = new QPushButton("Insert New Cylinder Marker");
    layout->addWidget( insert_cylinder_button_ );

    insert_torus_button_ = new QPushButton("Insert New Torus Marker");
    layout->addWidget( insert_torus_button_ );

    QHBoxLayout* name_layout = new QHBoxLayout;
    name_layout->addWidget( new QLabel( "Name:" ));
    name_editor_ = new QLineEdit;
    name_layout->addWidget( name_editor_ );
    layout->addLayout( name_layout );

    QHBoxLayout* description_layout = new QHBoxLayout;
    description_layout->addWidget( new QLabel( "Description:" ));
    description_editor_ = new QLineEdit;
    description_layout->addWidget( description_editor_ );
    layout->addLayout( description_layout );

    QHBoxLayout* frame_layout = new QHBoxLayout;
    frame_layout->addWidget( new QLabel( "Frame:" ));
    frame_editor_ = new QLineEdit;
    frame_layout->addWidget( frame_editor_ );
    layout->addLayout( frame_layout );

    erase_with_id_button_ = new QPushButton("Erase with id");
    layout->addWidget( erase_with_id_button_ );

    QHBoxLayout* id_layout = new QHBoxLayout;
    id_layout->addWidget( new QLabel( "Id:" ));
    id_editor_ = new QLineEdit;
    id_layout->addWidget( id_editor_ );
    layout->addLayout( id_layout );

    erase_all_button_ = new QPushButton("Erase all");
    layout->addWidget( erase_all_button_ );

    erase_focus_button_ = new QPushButton("Erase focus");
    layout->addWidget( erase_focus_button_ );

    setLayout( layout );

    connect( insert_box_button_, SIGNAL( clicked() ), this, SLOT( insertBoxService ()));
    connect( insert_cylinder_button_, SIGNAL( clicked() ), this, SLOT( insertCylinderService ()));
    connect( insert_torus_button_, SIGNAL( clicked() ), this, SLOT( insertTorusService ()));
    connect( erase_with_id_button_, SIGNAL( clicked() ), this, SLOT( eraseWithIdService ()));
    connect( erase_all_button_, SIGNAL( clicked() ), this, SLOT( eraseAllService ()));
    connect( erase_focus_button_, SIGNAL( clicked() ), this, SLOT( eraseFocusService ()));
  }

  void TransformableMarkerOperatorAction::insertBoxService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_interactive_marker::TransformableMarkerOperate::BOX;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertCylinderService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_interactive_marker::TransformableMarkerOperate::CYLINDER;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertTorusService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_interactive_marker::TransformableMarkerOperate::TORUS;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseWithIdService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::ERASE;
    operator_srv.request.operate.name = id_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseAllService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::ERASEALL;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseFocusService(){
    jsk_interactive_marker::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_interactive_marker::TransformableMarkerOperate::ERASEFOCUS;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::callRequestMarkerOperateService(jsk_interactive_marker::RequestMarkerOperate srv){
    ros::ServiceClient client = nh_.serviceClient<jsk_interactive_marker::RequestMarkerOperate>("request_marker_operate", true);
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  }

  void TransformableMarkerOperatorAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  // Load all configuration data for this panel from the given Config object.
  void TransformableMarkerOperatorAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::TransformableMarkerOperatorAction, rviz::Panel )
