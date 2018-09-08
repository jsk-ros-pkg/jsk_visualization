#include <iostream>

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QLabel>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>

#include <resource_retriever/retriever.h>
#include <jsk_interactive_marker/GetMarkerDimensions.h>
#include <jsk_interactive_marker/GetTransformableMarkerFocus.h>

#include "jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h"

using namespace rviz;
namespace jsk_interactive_marker
{
  TransformableMarkerOperatorAction::TransformableMarkerOperatorAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    // server name
    QHBoxLayout* server_name_layout = new QHBoxLayout;
    server_name_layout->addWidget( new QLabel( "Server Name:" ));
    server_name_editor_ = new QLineEdit;
    server_name_layout->addWidget( server_name_editor_ );
    layout->addLayout( server_name_layout );

    // topic name
    QHBoxLayout* obj_array_topic_layout = new QHBoxLayout;
    obj_array_topic_layout->addWidget( new QLabel( "ObjectArray Topic:" ));
    topic_name_editor_ = new QLineEdit;
    obj_array_topic_layout->addWidget( topic_name_editor_ );
    layout->addLayout( obj_array_topic_layout );

    // tabs for operations
    QTabWidget* tabs = new QTabWidget();

    QVBoxLayout* layout1 = new QVBoxLayout;
    QVBoxLayout* layout2 = new QVBoxLayout;
    QVBoxLayout* layout3 = new QVBoxLayout;

    QWidget* tab_1 = new QWidget();
    QWidget* tab_2 = new QWidget();
    QWidget* tab_3 = new QWidget();

    // tab_1, layout1: insert

    insert_box_button_ = new QPushButton("Insert New Box Marker");
    layout1->addWidget( insert_box_button_ );

    insert_cylinder_button_ = new QPushButton("Insert New Cylinder Marker");
    layout1->addWidget( insert_cylinder_button_ );

    insert_torus_button_ = new QPushButton("Insert New Torus Marker");
    layout1->addWidget( insert_torus_button_ );

    insert_mesh_button_ = new QPushButton("Insert New Mesh Marker");
    layout1->addWidget( insert_mesh_button_ );

    QHBoxLayout* object_layout = new QHBoxLayout;
    object_layout->addWidget( new QLabel( "Object:" ));
    object_editor_ = new QComboBox;
    object_editor_->setIconSize(QSize(50, 50));
    object_editor_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    object_layout->addWidget( object_editor_ );
    layout1->addLayout( object_layout );

    QHBoxLayout* name_layout = new QHBoxLayout;
    name_layout->addWidget( new QLabel( "Name:" ));
    name_editor_ = new QLineEdit;
    name_layout->addWidget( name_editor_ );
    layout1->addLayout( name_layout );

    QHBoxLayout* description_layout = new QHBoxLayout;
    description_layout->addWidget( new QLabel( "Description:" ));
    description_editor_ = new QLineEdit;
    description_layout->addWidget( description_editor_ );
    layout1->addLayout( description_layout );

    QHBoxLayout* frame_layout = new QHBoxLayout;
    frame_layout->addWidget( new QLabel( "Frame:" ));
    frame_editor_ = new QLineEdit;
    frame_layout->addWidget( frame_editor_ );
    layout1->addLayout( frame_layout );

    // tab_2, layout2: transform
    QVBoxLayout* transform_layout = new QVBoxLayout;
    transform_layout->addWidget( new QLabel( "Object Name:" ));
    transform_name_editor_ = new QLineEdit;
    transform_layout->addWidget( transform_name_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension X:" ));
    dimension_x_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_x_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Y:" ));
    dimension_y_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_y_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Z:" ));
    dimension_z_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_z_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Radius:" ));
    dimension_radius_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_radius_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Small Radius:" ));
    dimension_sm_radius_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_sm_radius_editor_ );
    layout2->addLayout( transform_layout );

    // tab_3, layout3: erase

    erase_with_id_button_ = new QPushButton("Erase with id");
    layout3->addWidget( erase_with_id_button_ );

    QHBoxLayout* id_layout = new QHBoxLayout;
    id_layout->addWidget( new QLabel( "Id:" ));
    id_editor_ = new QLineEdit;
    id_layout->addWidget( id_editor_ );
    layout3->addLayout( id_layout );

    erase_all_button_ = new QPushButton("Erase all");
    layout3->addWidget( erase_all_button_ );

    erase_focus_button_ = new QPushButton("Erase focus");
    layout3->addWidget( erase_focus_button_ );

    tab_1->setLayout( layout1 );
    tab_2->setLayout( layout2 );
    tab_3->setLayout( layout3 );

    tabs->addTab(tab_1, QString("Insert"));
    tabs->addTab(tab_2, QString("Transform"));
    tabs->addTab(tab_3, QString("Erase"));

    layout->addWidget( tabs );
    setLayout( layout );

    connect( insert_box_button_, SIGNAL( clicked() ), this, SLOT( insertBoxService ()));
    connect( insert_cylinder_button_, SIGNAL( clicked() ), this, SLOT( insertCylinderService ()));
    connect( insert_torus_button_, SIGNAL( clicked() ), this, SLOT( insertTorusService ()));
    connect( insert_mesh_button_, SIGNAL( clicked() ), this, SLOT( insertMeshService ()));
    connect( erase_with_id_button_, SIGNAL( clicked() ), this, SLOT( eraseWithIdService ()));
    connect( erase_all_button_, SIGNAL( clicked() ), this, SLOT( eraseAllService ()));
    connect( erase_focus_button_, SIGNAL( clicked() ), this, SLOT( eraseFocusService ()));

    connect( dimension_x_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_y_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_z_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_radius_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_sm_radius_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( object_editor_, SIGNAL( currentIndexChanged(int)), SLOT( updateName ()));
    connect( topic_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateObjectArrayTopic ()));
  }

  void TransformableMarkerOperatorAction::objectArrayCb(const jsk_recognition_msgs::ObjectArray::ConstPtr& obj_array_msg) {
    objects_ = obj_array_msg->objects;
    int current_index = object_editor_->currentIndex();
    object_editor_->clear();
    for (size_t i = 0; i < obj_array_msg->objects.size(); i++) {
      jsk_recognition_msgs::Object object =  objects_[i];
      // thumbnail
      QPixmap pixmap = QPixmap();
      if (object.image_resources.size() > 0)
      {
        std::string thumbnail = object.image_resources[0];
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource mem = retriever.get(thumbnail);
        pixmap.loadFromData(static_cast<unsigned char*>(mem.data.get()), mem.size);
      }
      // name
      std::stringstream ss;
      ss << object.id << ": " << object.name;
      //
      object_editor_->addItem(QIcon(pixmap), QString::fromStdString(ss.str()));
    }
    object_editor_->setCurrentIndex(current_index);
  }

  void TransformableMarkerOperatorAction::onInitialize() {
    connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));
    updateObjectArrayTopic();
  }

  void TransformableMarkerOperatorAction::update() {
    updateServerName();
    updateFocusMarkerDimensions();
    updateFrameId();
    // updateDimensionsService();
  }

  void TransformableMarkerOperatorAction::updateObjectArrayTopic() {
    sub_obj_array_.shutdown();
    std::string topic = topic_name_editor_->text().toStdString();
    if (topic.empty()) {
      ros::master::V_TopicInfo topics;
      ros::master::getTopics(topics);
      for (size_t i = 0; i < topics.size(); i++) {
        if (topics[i].datatype == "jsk_recognition_msgs/ObjectArray") {
          topic = topics[i].name;
          break;
        }
      }
      topic_name_editor_->setText(QString::fromStdString(topic));
    }
    sub_obj_array_ = nh_.subscribe(
      topic, 1, &TransformableMarkerOperatorAction::objectArrayCb, this);
  }

  void TransformableMarkerOperatorAction::insertBoxService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::BOX;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertCylinderService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertMeshService() {
    int current_index = object_editor_->currentIndex();
    if (!(0 <= current_index && current_index < objects_.size())) {
      ROS_ERROR("Invalid index for object selection: %d. Please select again.", current_index);
      return;
    }
    jsk_recognition_msgs::Object object = objects_[current_index];
    if (object.mesh_resource.empty()) {
      ROS_ERROR("Mesh resource of object '%s' is empty, so skipping.", object.name.c_str());
      return;
    }

    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = object.name;
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    operator_srv.request.operate.mesh_resource = object.mesh_resource;
    operator_srv.request.operate.mesh_use_embedded_materials = true;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::insertTorusService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.type = jsk_rviz_plugins::TransformableMarkerOperate::TORUS;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::INSERT;
    operator_srv.request.operate.name = name_editor_->text().toStdString();
    operator_srv.request.operate.description = description_editor_->text().toStdString();
    operator_srv.request.operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseWithIdService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASE;
    operator_srv.request.operate.name = id_editor_->text().toStdString();
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseAllService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASEALL;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::eraseFocusService(){
    jsk_rviz_plugins::RequestMarkerOperate operator_srv;
    operator_srv.request.operate.action = jsk_rviz_plugins::TransformableMarkerOperate::ERASEFOCUS;
    callRequestMarkerOperateService(operator_srv);
  };

  void TransformableMarkerOperatorAction::callRequestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate srv){
    std::string server_name = server_name_editor_->text().toStdString();
    std::string service_name = server_name + "/request_marker_operate";
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::RequestMarkerOperate>(service_name, true);
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL: %s", service_name.c_str());
    };
  }

  void TransformableMarkerOperatorAction::updateDimensionsService() {
    std::string server_name = server_name_editor_->text().toStdString();
    std::string service_name = server_name + "/set_dimensions";
    ros::ServiceClient client = nh_.serviceClient<jsk_interactive_marker::SetMarkerDimensions>(service_name, true);

    jsk_interactive_marker::SetMarkerDimensions srv;
    if (transform_name_editor_->text().toStdString().empty()) {
      srv.request.dimensions.x = transform_name_editor_->placeholderText().toFloat();
    } else {
      srv.request.dimensions.x = transform_name_editor_->text().toFloat();
    }
    if (dimension_x_editor_->text().toStdString().empty()) {
      srv.request.dimensions.x = dimension_x_editor_->placeholderText().toFloat();
    } else {
      srv.request.dimensions.x = dimension_x_editor_->text().toFloat();
    }
    if (dimension_y_editor_->text().toStdString().empty()) {
      srv.request.dimensions.y = dimension_y_editor_->placeholderText().toFloat();
    } else {
      srv.request.dimensions.y = dimension_y_editor_->text().toFloat();
    }
    if (dimension_z_editor_->text().toStdString().empty()) {
      srv.request.dimensions.z = dimension_z_editor_->placeholderText().toFloat();
    } else {
      srv.request.dimensions.z = dimension_z_editor_->text().toFloat();
    }

    if (client.call(srv)) {
      ROS_INFO("Call success: %s", service_name.c_str());
    } else {
      ROS_ERROR("Service call fail: %s", service_name.c_str());
    }
  }

  void TransformableMarkerOperatorAction::updateFrameId() {
    if (frame_editor_->text().isEmpty()) {
      frame_editor_->setText(vis_manager_->getFixedFrame());
    }
  }

  void TransformableMarkerOperatorAction::updateName() {
    int current_index = object_editor_->currentIndex();
    if (0 <= current_index && current_index < objects_.size()) {
      jsk_recognition_msgs::Object object = objects_[current_index];
      name_editor_->setText(QString::fromStdString(object.name));
    } else {
      name_editor_->setText(QString(""));
    }
  }

  void TransformableMarkerOperatorAction::updateServerName() {
    std::string server_name = server_name_editor_->text().toStdString();
    if (server_name.empty() && !ros::service::exists("/request_marker_operate", false)) {
      ros::V_string nodes;
      ros::master::getNodes(nodes);
      for (size_t i=0; i<nodes.size(); i++) {
        if (ros::service::exists(nodes[i] + "/request_marker_operate", false)) {
          server_name_editor_->setText(QString::fromStdString(nodes[i]));
          break;
        }
      }
    }
  }

  void TransformableMarkerOperatorAction::updateFocusMarkerDimensions() {
    std::string server_name = server_name_editor_->text().toStdString();
    ros::ServiceClient client_focus = nh_.serviceClient<jsk_interactive_marker::GetTransformableMarkerFocus>(
      server_name + "/get_focus", true);
    jsk_interactive_marker::GetTransformableMarkerFocus srv_focus;
    ros::ServiceClient client_dim = nh_.serviceClient<jsk_interactive_marker::GetMarkerDimensions>(
      server_name + "/get_dimensions", true);
    jsk_interactive_marker::GetMarkerDimensions srv_dim;
    if (client_focus.call(srv_focus) && client_dim.call(srv_dim)) {
      transform_name_editor_->setPlaceholderText(QString::fromStdString(srv_focus.response.target_name));
      dimension_x_editor_->setPlaceholderText(QString::number(srv_dim.response.dimensions.x, 'f', 4));
      dimension_y_editor_->setPlaceholderText(QString::number(srv_dim.response.dimensions.y, 'f', 4));
      dimension_z_editor_->setPlaceholderText(QString::number(srv_dim.response.dimensions.z, 'f', 4));
      dimension_radius_editor_->setPlaceholderText(QString::number(srv_dim.response.dimensions.radius, 'f', 4));
      dimension_sm_radius_editor_->setPlaceholderText(
        QString::number(srv_dim.response.dimensions.small_radius, 'f', 4));
    } else{
      ROS_ERROR_THROTTLE(10, "Service call FAIL: %s", server_name.c_str());
    }
  }

  void TransformableMarkerOperatorAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "ServerName", server_name_editor_->text().toStdString().c_str() );
  }

  void TransformableMarkerOperatorAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    QString server_name;
    config.mapGetString( "ServerName", &server_name );
    server_name_editor_->setText(server_name);
  }
}  // namespace jsk_interactive_marker

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_interactive_marker::TransformableMarkerOperatorAction, rviz::Panel )
