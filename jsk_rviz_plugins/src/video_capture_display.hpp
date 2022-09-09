// // -*- mode: c++ -*-
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

// #ifndef JSK_RVIZ_PLUGIN_VIDEO_CAPTURE_DISPLAY_H_
// #define JSK_RVIZ_PLUGIN_VIDEO_CAPTURE_DISPLAY_H_

// #include <rviz_common/display.hpp>
// #include <rviz_common/properties/string_property.hpp>
// #include <rviz_common/properties/bool_property.hpp>
// #include <rviz_common/properties/float_property.hpp>
// #include <rviz_common/properties/int_property.hpp>
// #include <opencv2/opencv.hpp>

// // #include <ros/ros.hpp>

// namespace jsk_rviz_plugins
// {
//   class VideoCaptureDisplay: public rviz_common::Display
//   {
//     Q_OBJECT
//   public:
//     typedef std::shared_ptr<VideoCaptureDisplay> Ptr;

//     VideoCaptureDisplay();
//     ~VideoCaptureDisplay();
//   protected:
//     void onInitialize() override;
//     void onEnable() override;
//     void onDisable() override {}
//     void update(float wall_dt, float ros_dt) override;
//     void reset(); // override;
//     void startCapture();
//     void stopCapture();
//     ////////////////////////////////////////////////////////
//     // Variables
//     ////////////////////////////////////////////////////////
//     rviz_common::properties::StringProperty* file_name_property_;
//     rviz_common::properties::BoolProperty* start_capture_property_;
//     rviz_common::properties::FloatProperty* fps_property_;
//     rviz_common::properties::BoolProperty* use_3d_viewer_size_property_;
//     rviz_common::properties::IntProperty* width_property_;
//     rviz_common::properties::IntProperty* height_property_;
//     std::string file_name_;
//     bool capturing_;
//     double fps_;
//     bool use_3d_viewer_size_;
//     int width_;
//     int height_;
//     int frame_counter_;
//     bool first_time_;
//     cv::VideoWriter writer_;
//   protected Q_SLOTS:
//     void updateFileName();
//     void updateStartCapture();
//     void updateFps();
//     void updateUse3DViewerSize();
//     void updateWidth();
//     void updateHeight();
//   private:

//   };
// }

// #endif
