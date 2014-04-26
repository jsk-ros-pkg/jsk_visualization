// -*- mode: C++ -*-
#include "normal_display.h"

using namespace rviz;

namespace jsk_rviz_plugin
{

  NormalDisplay::NormalDisplay()
  {
    style_property_ = new EnumProperty( "Style", "PointsColor",
                                        "Rendering mode to use, in order of computational complexity.",
                                        this, SLOT( updateStyle() ), this);
    style_property_->addOption( "PointsColor", NormalDisplay::POINTS_COLOR );
    style_property_->addOption( "FlatColor", NormalDisplay::FLAT_COLOR );
    style_property_->addOption( "DirectionColor", NormalDisplay::DIRECTION_COLOR );

    color_property_ = new ColorProperty( "Color", Qt::white,
                                         "Color to assign to every point.",this);
    color_property_->hide();
  }

  void NormalDisplay::updateStyle()
  {
    NormalDisplay::ColorTypes mode = (NormalDisplay::ColorTypes) style_property_->getOptionInt();
    if( mode != NormalDisplay::FLAT_COLOR )
      {
        color_property_->hide();
      }
    else
      {
        color_property_->show();
      }
  }

  void NormalDisplay::onInitialize()
  {
    MFDClass::onInitialize();
  }

  NormalDisplay::~NormalDisplay()
  {
    delete style_property_;
    delete color_property_;
    visuals_.clear();
  }

  void NormalDisplay::reset()
  {
    MFDClass::reset();
    visuals_.clear();
  }


  void NormalDisplay::processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg )
  {
    ROS_INFO("Here");
    //check x,y,z
    int32_t xi = findChannelIndex(msg, "x");
    int32_t yi = findChannelIndex(msg, "y");
    int32_t zi = findChannelIndex(msg, "z");

    if (xi == -1 || yi == -1 || zi == -1)
      {
        ROS_ERROR("doesn't have x, y, z");
        return;
      }

    const uint32_t xoff = msg->fields[xi].offset;
    const uint32_t yoff = msg->fields[yi].offset;
    const uint32_t zoff = msg->fields[zi].offset;

    //check normals x,y,z
    int32_t normal_xi = findChannelIndex(msg, "normal_x");
    int32_t normal_yi = findChannelIndex(msg, "normal_y");
    int32_t normal_zi = findChannelIndex(msg, "normal_z");

    if (normal_xi == -1 || normal_yi == -1 || normal_zi == -1)
      {
        ROS_ERROR("doesn't have normal_x, normal_y, normal_z");
        return;
      }

    const uint32_t normal_xoff = msg->fields[normal_xi].offset;
    const uint32_t normal_yoff = msg->fields[normal_yi].offset;
    const uint32_t normal_zoff = msg->fields[normal_zi].offset;

    //check rgba color
    int32_t rgbai = findChannelIndex(msg, "rgb");
    uint32_t rgbaoff = -1;
    if(rgbai != -1)
      rgbaoff = msg->fields[rgbai].offset;


    //check other option values
    const uint32_t point_step = msg->point_step;
    const size_t point_count = msg->width * msg->height;

    if (point_count == 0)
      {
        ROS_ERROR("doesn't have point_count > 0");
        return;
      }


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

    visuals_.rset_capacity(int(point_count/100));
    const uint8_t* ptr = &msg->data.front();
    for (size_t i = 0; i < point_count; ++i)
      {
        if(i %100 != 0){
          ptr += point_step;
          continue;
        }
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);
        float normal_x = *reinterpret_cast<const float*>(ptr + normal_xoff);
        float normal_y = *reinterpret_cast<const float*>(ptr + normal_yoff);
        float normal_z = *reinterpret_cast<const float*>(ptr + normal_zoff);
        int r=1,g=0,b=0;

        if (validateFloats(Ogre::Vector3(x, y, z)) && validateFloats(Ogre::Vector3(normal_x, normal_y, normal_z)))
          {
            boost::shared_ptr<NormalVisual> visual;
            if(visuals_.full()){
              visual = visuals_.front();
            }else{
              visual.reset(new NormalVisual( context_->getSceneManager(), scene_node_ ));
            }
            visual->setValues( x, y, z, normal_x, normal_y, normal_z );
            visual->setFramePosition( position );
            visual->setFrameOrientation( orientation );

            QColor color = color_property_->getColor();
            Ogre::Vector3 dir_vec(normal_x, normal_y, normal_z);
            switch((NormalDisplay::ColorTypes) style_property_->getOptionInt()){
            case (NormalDisplay::POINTS_COLOR):
              if(rgbai != -1){
                b = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff);
                g = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff + 1*sizeof(uint8_t));
                r = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff + 2*sizeof(uint8_t));
              }
              visual->setColor( r/256.0, g/256.0, b/256.0, 1 );
              break;
            case (NormalDisplay::FLAT_COLOR):
              visual->setColor(color.redF(), color.greenF(), color.blueF(), 1);
              break;
            case (NormalDisplay::DIRECTION_COLOR):
              visual->setColor( dir_vec.dotProduct(Ogre::Vector3(1,0,0)), dir_vec.dotProduct(Ogre::Vector3(0,1,0)), dir_vec.dotProduct(Ogre::Vector3(0,0,-1)), 1 );
              break;
            }
            visuals_.push_back(visual);
            ROS_INFO("normal valus %f %f %f %f %f %f / %d %d %d", x, y, z, normal_x, normal_y, normal_z, r, g, b);
          }else{
          ROS_ERROR("invalid floats are inputed");
        }

        ptr += point_step;
      }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::NormalDisplay,rviz::Display )
