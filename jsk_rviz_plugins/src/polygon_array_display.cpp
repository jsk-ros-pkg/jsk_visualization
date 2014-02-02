#include "polygon_array_display.h"
#include "rviz/properties/parse_color.h"
#include <rviz/validate_floats.h>

namespace jsk_rviz_plugin
{
  PolygonArrayDisplay::PolygonArrayDisplay()
  {
    color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
                                               "Color to draw the polygons.",
                                               this, SLOT( queueRender() ));
    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "Amount of transparency to apply to the polygon.",
                                               this, SLOT( queueRender() ));
    alpha_property_->setMin( 0 );
    alpha_property_->setMax( 1 );
  }
  
  PolygonArrayDisplay::~PolygonArrayDisplay()
  {
    delete alpha_property_;
    delete color_property_;
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      scene_manager_->destroyManualObject(manual_objects_[i]);
      scene_manager_->destroySceneNode(scene_nodes_[i]);
    }
  }
  
  void PolygonArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    
  }

  bool validateFloats( const jsk_pcl_ros::PolygonArray& msg)
  {
    for (size_t i = 0; i < msg.polygons.size(); i++) {
      if (!rviz::validateFloats(msg.polygons[i].polygon.points))
        return false;
    }
    return true;
  }
  
  void PolygonArrayDisplay::reset()
  {
    MFDClass::reset();
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      manual_objects_[i]->clear();
    }
  }

  void PolygonArrayDisplay::processMessage(const jsk_pcl_ros::PolygonArray::ConstPtr& msg)
  {
    if (!validateFloats(*msg)) {
      setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
      return;
    }

    // create nodes and manual objects
    if (msg->polygons.size() > manual_objects_.size()) {
      for (size_t i = manual_objects_.size(); i < msg->polygons.size(); i++) {
        Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
        Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic( true );
        scene_node->attachObject( manual_object );
        
        manual_objects_.push_back(manual_object);
        scene_nodes_.push_back(scene_node);
      }
    }
    else if (msg->polygons.size() < manual_objects_.size()) {
      for (size_t i = msg->polygons.size(); i < manual_objects_.size(); i++) {
        scene_manager_->destroyManualObject( manual_objects_[i] );
        scene_manager_->destroySceneNode( scene_nodes_[i] );
      }
      // resize the array
      manual_objects_.resize(msg->polygons.size());
      scene_nodes_.resize(msg->polygons.size());
    }
    
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = msg->polygons[i];
      Ogre::SceneNode* scene_node = scene_nodes_[i];
      Ogre::ManualObject* manual_object = manual_objects_[i];
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if( !context_->getFrameManager()->getTransform( polygon.header, position, orientation )) {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   polygon.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      }
        scene_node->setPosition( position );
        scene_node->setOrientation( orientation );
        manual_object->clear();
        Ogre::ColourValue color = rviz::qtToOgre( color_property_->getColor() );
        color.a = alpha_property_->getFloat();
        uint32_t num_points = polygon.polygon.points.size();
        if( num_points > 0 )
        {
          manual_object->estimateVertexCount( num_points );
          manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
          for( uint32_t i = 0; i < num_points + 1; ++i )
          {
            const geometry_msgs::Point32& msg_point = polygon.polygon.points[ i % num_points ];
            manual_object->position( msg_point.x, msg_point.y, msg_point.z );
            manual_object->colour( color );
          }
          
          manual_object->end();
        }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::PolygonArrayDisplay, rviz::Display )
