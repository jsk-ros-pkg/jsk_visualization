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
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "PolygonArray" << count++;
    ss << "Material";
    material_name_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create(material_name_, "rviz");
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
  }
  
  PolygonArrayDisplay::~PolygonArrayDisplay()
  {
    delete alpha_property_;
    delete color_property_;
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
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
    if (msg->polygons.size() * 2 > manual_objects_.size()) {
      for (size_t i = manual_objects_.size(); i < msg->polygons.size() * 2; i++) {
        Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
        Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic( true );
        scene_node->attachObject( manual_object );
        
        manual_objects_.push_back(manual_object);
        scene_nodes_.push_back(scene_node);
      }
    }
    else if (msg->polygons.size() * 2 < manual_objects_.size()) {
      for (size_t i = msg->polygons.size() * 2; i < manual_objects_.size(); i++) {
        scene_manager_->destroyManualObject( manual_objects_[i] );
        scene_manager_->destroySceneNode( scene_nodes_[i] );
      }
      // resize the array
      manual_objects_.resize(msg->polygons.size() * 2);
      scene_nodes_.resize(msg->polygons.size() * 2);
    }

    Ogre::ColourValue color = rviz::qtToOgre( color_property_->getColor() );
    color.a = alpha_property_->getFloat();
    material_->getTechnique(0)->setAmbient( color * 0.5 );
    material_->getTechnique(0)->setDiffuse( color );
    if ( color.a < 0.9998 )
    {
      material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
      material_->getTechnique(0)->setDepthWriteEnabled( false );
    }
    else
    {
      material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
      material_->getTechnique(0)->setDepthWriteEnabled( true );
    }
    
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = msg->polygons[i];
      Ogre::SceneNode* scene_node = scene_nodes_[i * 2];
      Ogre::ManualObject* manual_object = manual_objects_[i * 2];
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if( !context_->getFrameManager()->getTransform( polygon.header, position, orientation )) {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   polygon.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      }
      scene_node->setPosition( position );
      scene_node->setOrientation( orientation );
      manual_object->clear();
        
      uint32_t num_points = polygon.polygon.points.size();
      if( num_points > 0 )
      {
        manual_object->estimateVertexCount( num_points );
        manual_object->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_FAN );
        for( uint32_t i = 0; i < num_points + 1; ++i )
        {
          const geometry_msgs::Point32& msg_point = polygon.polygon.points[ i % num_points ];
          manual_object->position( msg_point.x, msg_point.y, msg_point.z );
        }
          
        manual_object->end();
      }
    }
    // reverse order
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = msg->polygons[i];
      Ogre::SceneNode* scene_node = scene_nodes_[i * 2 + 1];
      Ogre::ManualObject* manual_object = manual_objects_[i * 2 + 1];
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if( !context_->getFrameManager()->getTransform( polygon.header, position, orientation )) {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   polygon.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      }
      scene_node->setPosition( position );
      scene_node->setOrientation( orientation );
      manual_object->clear();
        
      uint32_t num_points = polygon.polygon.points.size();
      if( num_points > 0 )
      {
        manual_object->estimateVertexCount( num_points );
        manual_object->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_FAN );
        for( uint32_t i = num_points; i > 0; --i )
        {
          const geometry_msgs::Point32& msg_point = polygon.polygon.points[ i % num_points ];
          manual_object->position( msg_point.x, msg_point.y, msg_point.z );
        }
          
        manual_object->end();
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::PolygonArrayDisplay, rviz::Display )
