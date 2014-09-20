#include "polygon_array_display.h"
#include "rviz/properties/parse_color.h"
#include <rviz/validate_floats.h>
#include <jsk_topic_tools/color_utils.h>

namespace jsk_rviz_plugin
{
  PolygonArrayDisplay::PolygonArrayDisplay()
  {
    auto_coloring_property_ = new rviz::BoolProperty("auto color", true,
                                                      "automatically change the color of the polygons",
                                                      this, SLOT(updateAutoColoring()));
    color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
                                               "Color to draw the polygons.",
                                               this, SLOT( queueRender() ));
    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "Amount of transparency to apply to the polygon.",
                                               this, SLOT( queueRender() ));
    // only_border_property_ = new rviz::BoolProperty("only border", true,
    //                                                "only shows the borders of polygons",
    //                                                this, SLOT(updateOnlyBorder()));
    only_border_ = true;
    alpha_property_->setMin( 0 );
    alpha_property_->setMax( 1 );
  }
  
  PolygonArrayDisplay::~PolygonArrayDisplay()
  {
    delete alpha_property_;
    delete color_property_;
    //delete only_border_property_;
    delete auto_coloring_property_;

    for (size_t i = 0; i < lines_.size(); i++) {
      delete lines_[i];
    }
    
    for (size_t i = 0; i < materials_.size(); i++) {
      materials_[i]->unload();
      Ogre::MaterialManager::getSingleton().remove(materials_[i]->getName());
    }
    
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      scene_manager_->destroyManualObject(manual_objects_[i]);
      scene_manager_->destroySceneNode(scene_nodes_[i]);
    }
  }
  
  void PolygonArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    //updateOnlyBorder();
    updateAutoColoring();
  }

  void PolygonArrayDisplay::allocateMaterials(int num)
  {
    if (only_border_) {
      return;
    }
    static uint32_t count = 0;
    
    if (num > materials_.size()) {
      for (size_t i = materials_.size(); num > i; i++) {
        std::stringstream ss;
        ss << "PolygonArrayMaterial" << count++;
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(true);
        material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
        materials_.push_back(material);
      }
    }
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

  void PolygonArrayDisplay::updateSceneNodes(const jsk_pcl_ros::PolygonArray::ConstPtr& msg)
  {
    int scale_factor = 2;
    if (only_border_) {
      scale_factor = 1;
    }
    if (msg->polygons.size() * scale_factor > manual_objects_.size()) {
      for (size_t i = manual_objects_.size(); i < msg->polygons.size() * scale_factor; i++) {
        Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
        Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic( true );
        scene_node->attachObject( manual_object );
        manual_objects_.push_back(manual_object);
        scene_nodes_.push_back(scene_node);
      }
    }
    else if (msg->polygons.size() * scale_factor < manual_objects_.size()) {
      for (size_t i = msg->polygons.size() * scale_factor; i < manual_objects_.size(); i++) {
        manual_objects_[i]->setVisible(false);
      }
    }
  }

  void PolygonArrayDisplay::updateLines(int num)
  {
    if (num > lines_.size()) {
      for (size_t i = lines_.size(); i < num; i++) {
        rviz::BillboardLine* line = new rviz::BillboardLine(context_->getSceneManager(), scene_nodes_[i]);
        line->setLineWidth(0.01);
        line->setNumLines(1);
        lines_.push_back(line);
      }
    }
    for (size_t i = 0; i < lines_.size(); i++) {
      lines_[i]->clear();
    }
  }
  
  void PolygonArrayDisplay::processMessage(const jsk_pcl_ros::PolygonArray::ConstPtr& msg)
  {
    if (!validateFloats(*msg)) {
      setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
      return;
    }
    // create nodes and manual objects
    updateSceneNodes(msg);
    allocateMaterials(msg->polygons.size());
    updateLines(msg->polygons.size());
    
    if (only_border_) {
      // use line_
      for (size_t i = 0; i < manual_objects_.size(); i++) {
        manual_objects_[i]->setVisible(false);
      }
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = msg->polygons[i];
        if (polygon.polygon.points.size() >= 3) {
          Ogre::SceneNode* scene_node = scene_nodes_[i];
          //Ogre::ManualObject* manual_object = manual_objects_[i];
          Ogre::Vector3 position;
          Ogre::Quaternion orientation;
          if( !context_->getFrameManager()->getTransform( polygon.header, position, orientation )) {
            ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                       polygon.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
          }
          scene_node->setPosition( position );
          scene_node->setOrientation( orientation );
          rviz::BillboardLine* line = lines_[i];
          line->clear();
          line->setMaxPointsPerLine(polygon.polygon.points.size() + 1);
        
          Ogre::ColourValue color;
          if (auto_coloring_) {
            std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(i);
            color.r = ros_color.r;
            color.g = ros_color.g;
            color.b = ros_color.b;
            color.a = ros_color.a;
          }
          else {
            color = rviz::qtToOgre( color_property_->getColor() );
          }
          color.a = alpha_property_->getFloat();
          line->setColor(color.r, color.g, color.b, color.a);

          for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
            Ogre::Vector3 step_position;
            step_position.x = polygon.polygon.points[i].x;
            step_position.y = polygon.polygon.points[i].y;
            step_position.z = polygon.polygon.points[i].z;
            line->addPoint(step_position);
          }
          Ogre::Vector3 step_position;
          step_position.x = polygon.polygon.points[0].x;
          step_position.y = polygon.polygon.points[0].y;
          step_position.z = polygon.polygon.points[0].z;
          line->addPoint(step_position);
        }
      }
    }
    else {
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        Ogre::ColourValue color;
        if (auto_coloring_) {
          std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(i);
          color.r = ros_color.r;
          color.g = ros_color.g;
          color.b = ros_color.b;
          color.a = ros_color.a;
        }
        else {
          color = rviz::qtToOgre( color_property_->getColor() );
        }
        color.a = alpha_property_->getFloat();
        materials_[i]->getTechnique(0)->setAmbient( color * 0.5 );
        materials_[i]->getTechnique(0)->setDiffuse( color );
        if ( color.a < 0.9998 )
        {
          materials_[i]->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
          materials_[i]->getTechnique(0)->setDepthWriteEnabled( false );
        }
        else
        {
          materials_[i]->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
          materials_[i]->getTechnique(0)->setDepthWriteEnabled( true );
        }
      
        materials_[i]->getTechnique(0)->setAmbient( color * 0.5 );
        materials_[i]->getTechnique(0)->setDiffuse( color );
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
        manual_object->setVisible(true);
        uint32_t num_points = polygon.polygon.points.size();
        if( num_points > 0 )
        {
          manual_object->estimateVertexCount( num_points );
          manual_object->begin(materials_[i]->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN );
          for( uint32_t i = 0; i < num_points + 1; ++i )
          {
            const geometry_msgs::Point32& msg_point = polygon.polygon.points[ i % num_points ];
            manual_object->position( msg_point.x, msg_point.y, msg_point.z );
          }
          manual_object->end();
        }
      }
      //     reverse order
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
        manual_object->setVisible(false);
        manual_object->clear();
        
        uint32_t num_points = polygon.polygon.points.size();
        if( num_points > 0 )
        {
          manual_object->estimateVertexCount( num_points );
          manual_object->begin(materials_[i]->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN );
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

  void PolygonArrayDisplay::updateAutoColoring()
  {
    auto_coloring_ = auto_coloring_property_->getBool();
  }
  void PolygonArrayDisplay::updateOnlyBorder()
  {
    only_border_ = only_border_property_->getBool();
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::PolygonArrayDisplay, rviz::Display )
