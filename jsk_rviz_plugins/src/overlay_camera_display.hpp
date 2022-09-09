// // -*- mode: c++; -*-
// Copyright (c) 2014, JSK Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the JSK Lab nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// #ifndef JSK_RVIZ_PLUGINS_OVERLAY_CAMERA_DISPLAY_H_
// #define JSK_RVIZ_PLUGINS_OVERLAY_CAMERA_DISPLAY_H_

// #include <rviz_common/default_plugin/camera_display.hpp>

// #ifndef Q_MOC_RUN
// #include <QObject>

// #include <OgreMaterial.h>
// #include <OgreRenderTargetListener.h>
// #include <OgreSharedPtr.h>

// # include <sensor_msgs/CameraInfo.hpp>

// # include <message_filters/subscriber.hpp>
// #if ROS_VERSION_MINIMUM(1, 15, 0) // noetic and greater
// # include <tf2_ros/message_filter.hpp>
// #else
// # include <tf/message_filter.hpp>
// #endif

// # include <rviz/image/image_display_base.hpp>
// # include <rviz/image/ros_image_texture.hpp>
// # include <rviz/render_panel.hpp>

// #include "overlay_utils.hpp"
// #endif

// namespace Ogre
// {
// class SceneNode;
// class ManualObject;
// class Rectangle2D;
// class Camera;
// }

// namespace rviz
// {
// class EnumProperty;
// class FloatProperty;
// class IntProperty;
// class RenderPanel;
// class RosTopicProperty;
// class DisplayGroupVisibilityProperty;
// }
// /**
//  * \class CameraDisplay
//  *
//  */
// namespace jsk_rviz_plugins
// {
// using namespace rviz;
// class OverlayCameraDisplay: public rviz_common::properties::ImageDisplayBase, public Ogre::RenderTargetListener
// {
// Q_OBJECT
// public:
//   OverlayCameraDisplay();
//   ~OverlayCameraDisplay();

//   // Overrides from Display
//   void onInitialize() override;
//   void fixedFrameChanged();
//   void update( float wall_dt, float ros_dt );
//   void reset();

//   // Overrides from Ogre::RenderTargetListener
//   void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
//   void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

//   static const QString BACKGROUND;
//   static const QString OVERLAY;
//   static const QString BOTH;

// protected:
//   // overrides from Display
//   void onEnable() override;
//   void onDisable() override;

//   ROSImageTexture texture_;
//   RenderPanel* render_panel_;

// private Q_SLOTS:
//   void forceRender();
//   void updateAlpha();

//   void updateQueueSize();

// private:

//   void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
//   void caminfoCallback( const sensor_msgs::CameraInfo::ConstSharedPtr& msg );

//   bool updateCamera();

//   void clear();
//   void updateStatus();

//   Ogre::SceneNode* bg_scene_node_;
//   Ogre::SceneNode* fg_scene_node_;

//   Ogre::Rectangle2D* bg_screen_rect_;
//   Ogre::MaterialPtr bg_material_;

//   Ogre::Rectangle2D* fg_screen_rect_;
//   Ogre::MaterialPtr fg_material_;

//   message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
//   tf2_ros::MessageFilter<sensor_msgs::CameraInfo>* caminfo_tf_filter_;

//   FloatProperty* alpha_property_;
//   EnumProperty* image_position_property_;
//   FloatProperty* zoom_property_;
//   DisplayGroupVisibilityProperty* visibility_property_;

//   sensor_msgs::CameraInfo::ConstSharedPtr current_caminfo_;
//   std::mutex caminfo_mutex_;

//   bool new_caminfo_;

//   bool caminfo_ok_;

//   bool force_render_;

//   uint32_t vis_bit_;
// protected:
//   OverlayObject::Ptr overlay_;
//   void redraw();
//   rviz_common::properties::IntProperty* width_property_;
//   rviz_common::properties::IntProperty* height_property_;
//   rviz_common::properties::IntProperty* left_property_;
//   rviz_common::properties::IntProperty* top_property_;
//   rviz_common::properties::FloatProperty* texture_alpha_property_;
//   int width_, height_;
//   int left_, top_;
//   float texture_alpha_;
//   bool initializedp_;
// private Q_SLOTS:
//   void updateWidth();
//   void updateHeight();
//   void updateLeft();
//   void updateTop();
//   void updateTextureAlpha();
// };

// }

// #endif
