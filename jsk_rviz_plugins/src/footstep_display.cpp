#include "footstep_display.h"
#include <rviz/validate_floats.h>

namespace jsk_rviz_plugin
{
  FootstepDisplay::FootstepDisplay()
  {
    alpha_property_ =  new rviz::FloatProperty( "Alpha", 0.5,
                                                "0 is fully transparent, 1.0 is fully opaque.",
                                                this, SLOT( updateAlpha() ));
    width_property_ =  new rviz::FloatProperty( "Width", 0.23,
                                                "width of the footstep",
                                                this, SLOT( updateFootstepSize() ));
    height_property_ =  new rviz::FloatProperty( "height", 0.1,
                                                "height of the footstep",
                                                this, SLOT( updateFootstepSize() ));

    depth_property_ =  new rviz::FloatProperty( "depth", 0.3,
                                                "depth of the footstep",
                                                this, SLOT( updateFootstepSize() ));
  }

  FootstepDisplay::~FootstepDisplay()
  {
    delete alpha_property_;
    delete width_property_;
    delete height_property_;
    delete depth_property_;
    delete line_;
  }

  void FootstepDisplay::updateFootstepSize()
  {
    Ogre::Vector3 scale;
    scale[0] = depth_property_->getFloat();
    scale[1] = width_property_->getFloat();
    scale[2] = height_property_->getFloat();
    for (size_t i = 0; i < shapes_.size(); i++)
    {
      shapes_[i]->setScale(scale);
    }
  }
  
  void FootstepDisplay::updateAlpha()
  {
    float alpha = alpha_property_->getFloat();
    for (size_t i = 0; i < shapes_.size(); i++)
    {
      ShapePtr shape = shapes_[i];
      jsk_footstep_msgs::Footstep footstep = latest_footstep_->footsteps[i];
      if (footstep.leg == jsk_footstep_msgs::Footstep::LEFT)
      {
        shape->setColor(0, 1, 0, alpha);
      }
      else if (footstep.leg == jsk_footstep_msgs::Footstep::RIGHT)
      {
        shape->setColor(1, 0, 0, alpha);
      }
      else
      {
        shape->setColor(1, 1, 1, alpha);
      }
    }
      
  }
  
  void FootstepDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    line_->clear();
  }

  bool FootstepDisplay::validateFloats( const jsk_footstep_msgs::FootstepArray& msg )
  {
    for (std::vector<jsk_footstep_msgs::Footstep>::const_iterator it = msg.footsteps.begin();
         it != msg.footsteps.end();
         ++it) {
      if (!rviz::validateFloats((*it).pose.position.x)
          || !rviz::validateFloats((*it).pose.position.y)
          || !rviz::validateFloats((*it).pose.position.z)
          || !rviz::validateFloats((*it).pose.orientation.x)
          || !rviz::validateFloats((*it).pose.orientation.y)
          || !rviz::validateFloats((*it).pose.orientation.z)
          || !rviz::validateFloats((*it).pose.orientation.w)
        ) {
        return false;
      }
    }
    return true;
  }

  void FootstepDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    line_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
  }
  
  void FootstepDisplay::processMessage(const jsk_footstep_msgs::FootstepArray::ConstPtr& msg)
  {
    if (!validateFloats(*msg)) {
      setStatus(rviz::StatusProperty::Error, "Topic", "message contained invalid floating point values (nans or infs)");
      return;
    }
    latest_footstep_ = msg;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                    msg->header.stamp,
                                                    position, orientation ))
    {
      ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }

    // check thhe length of the shapes_
    if (msg->footsteps.size() > shapes_.size())
    {
      // need to allocate
      for (size_t i = shapes_.size(); i < msg->footsteps.size(); i++)
      {
        ShapePtr shape;
        shape.reset(new rviz::Shape(rviz::Shape::Cube, context_->getSceneManager(),
                                    scene_node_));
        shapes_.push_back(shape);
      }
    }
    else if (msg->footsteps.size() < shapes_.size())
    {
      // need to remove
      shapes_.resize(msg->footsteps.size());
    }

    line_->clear();
    line_->setLineWidth(0.01);
    line_->setNumLines(1);
    line_->setMaxPointsPerLine(1024);

    for (size_t i = 0; i < msg->footsteps.size(); i++)
    {
      ShapePtr shape = shapes_[i];
      jsk_footstep_msgs::Footstep footstep = msg->footsteps[i];
      Ogre::Vector3 step_position;
      Ogre::Quaternion step_quaternion;
      if( !context_->getFrameManager()->transform( msg->header, footstep.pose,
                                                   step_position,
                                                   step_quaternion ))
      {
        ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        return;
      }
      shape->setPosition(step_position);
      shape->setOrientation(step_quaternion);

      line_->addPoint(step_position);
      
    }
    updateFootstepSize();
    updateAlpha();
    context_->queueRender();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::FootstepDisplay, rviz::Display )
