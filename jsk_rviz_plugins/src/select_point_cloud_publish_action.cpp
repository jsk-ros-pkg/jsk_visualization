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

#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>
#include <rviz_common/properties/status_list.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include "select_point_cloud_publish_action.h"

namespace jsk_rviz_plugins
{

  SelectPointCloudPublishAction::SelectPointCloudPublishAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Button to send cancel topic
    publish_pointcloud_button_ = new QPushButton("SelectPointCloudPublish Action");
    layout->addWidget( publish_pointcloud_button_ );

    setLayout( layout );

    connect( publish_pointcloud_button_, SIGNAL( clicked() ), this, SLOT( publishPointCloud ()));
  }

  void SelectPointCloudPublishAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    select_pointcloud_publisher_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("selected_pointcloud", 1);
  }

  void SelectPointCloudPublishAction::publishPointCloud(){
    rviz_common::properties::PropertyTreeModel* model_ =
      getDisplayContext()->getSelectionManager()->getPropertyModel();
    int num_children = model_->rowCount();
    if( num_children > 0 )
      {
        RCLCPP_INFO(node_->get_logger(), "num > %d!", num_children);
        sensor_msgs::msg::PointCloud2 pc2;
        pc2.header.stamp = node_->now();
        pc2.header.frame_id = "camera_depth_optical_frame";
        pc2.height = 1;
        pc2.width  = num_children;

        pc2.fields.resize(4);
        pc2.fields[0].name = "x";
        pc2.fields[1].name = "y";
        pc2.fields[2].name = "z";
        pc2.fields[3].name = "rgb";
        pc2.fields[0].offset = 0;
        pc2.fields[1].offset = 4;
        pc2.fields[2].offset = 8;
        pc2.fields[3].offset = 12;
        pc2.fields[0].count =  pc2.fields[1].count =  pc2.fields[2].count =  pc2.fields[3].count = 1;
        pc2.fields[0].datatype =  pc2.fields[1].datatype =  pc2.fields[2].datatype =
          pc2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;

        pc2.data.resize(num_children * 4 * sizeof(float));
        for( int i = 0; i < num_children; i++ )
          {
            QModelIndex child_index = model_->index( i, 0, QModelIndex());
            rviz_common::properties::VectorProperty* vec_data =
              qobject_cast<rviz_common::properties::VectorProperty* >(
                model_->getProp( child_index )->childAt(0));
            rviz_common::properties::ColorProperty* color_data =
              qobject_cast<rviz_common::properties::ColorProperty* >(
                model_->getProp( child_index )->childAt(1));

            Ogre::Vector3 point_vec = vec_data->getVector();
            // check if color_data is available
            // if not color_data is available, set the color to black(0,0,0)
            int rgb_int = 0;
            if (color_data != NULL && color_data->getColor().isValid()) {
              Ogre::ColourValue point_color = color_data->getOgreColor();
              rgb_int = (int)point_color.r << 16 | (int)point_color.g << 8 |  (int)point_color.b << 0;
            }
            float x = point_vec.x, y = point_vec.y, z = point_vec.z;
            //Tty to add color, but point_color's value are all zero!!!!!!
            float rgb_float = *reinterpret_cast<float*>(&rgb_int);
            memcpy(&pc2.data[i*4*sizeof(float)], &x, sizeof(float));
            memcpy(&pc2.data[(i*4+1)*sizeof(float)], &y, sizeof(float));
            memcpy(&pc2.data[(i*4+2)*sizeof(float)], &z, sizeof(float));
            memcpy(&pc2.data[(i*4+3)*sizeof(float)], &rgb_float, sizeof(float));
          }

        pc2.point_step = 16;
        pc2.row_step = pc2.point_step * pc2.width;
        pc2.is_dense = false;
        select_pointcloud_publisher_->publish(pc2);
      }
  }

  void SelectPointCloudPublishAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
  }

  // Load all configuration data for this panel from the given Config object.
  void SelectPointCloudPublishAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::SelectPointCloudPublishAction, rviz_common::Panel )
