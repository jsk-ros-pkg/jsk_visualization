// -*- mode: c++ -*-
// Copyright (c) 2015, JSK Lab
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

#ifndef JSK_RVIZ_PLUGIN_OVERLAY_PICKER_TOOL_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_PICKER_TOOL_H_

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace jsk_rviz_plugins
{
class OverlayPickerTool : public rviz_common::Tool
{
public:
  OverlayPickerTool();
  void activate() {}
  void deactivate(){};
  // int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel);
  int processMouseEvent(rviz_common::ViewportMouseEvent & event);
  template <class T>
  T * isPropertyType(rviz_common::properties::Property * p)
  {
    try {
      return dynamic_cast<T *>(p);
    } catch (const std::bad_cast & e) {
      return NULL;
    }
  }

  template <class T>
  bool startMovement(
    rviz_common::properties::Property * property, rviz_common::ViewportMouseEvent & event,
    const std::string & type)
  {
    if (isPropertyType<T>(property)) {
      bool res = isPropertyType<T>(property)->isInRegion(event.x, event.y);
      if (res) {
        target_property_ = property;
        target_property_type_ = type;
        move_offset_x_ = event.x - isPropertyType<T>(property)->getX();
        move_offset_y_ = event.y - isPropertyType<T>(property)->getY();
      }
      return res;
    } else {
      return false;
    }
  }

  template <class T>
  void movePosition(rviz_common::ViewportMouseEvent & event)
  {
    if (shift_pressing_) {
      int orig_x = event.x - move_offset_x_;
      int orig_y = event.y - move_offset_y_;
      isPropertyType<T>(target_property_)->movePosition(20 * (orig_x / 20), 20 * (orig_y / 20));
    } else {
      isPropertyType<T>(target_property_)
        ->movePosition(event.x - move_offset_x_, event.y - move_offset_y_);
    }
  }

  template <class T>
  void setPosition(rviz_common::ViewportMouseEvent & event)
  {
    if (shift_pressing_) {
      int orig_x = event.x - move_offset_x_;
      int orig_y = event.y - move_offset_y_;
      isPropertyType<T>(target_property_)->setPosition(20 * (orig_x / 20), 20 * (orig_y / 20));
    } else {
      isPropertyType<T>(target_property_)
        ->setPosition(event.x - move_offset_x_, event.y - move_offset_y_);
    }
  }

protected:
  void onClicked(rviz_common::ViewportMouseEvent & event);
  void onMove(rviz_common::ViewportMouseEvent & event);
  void onRelease(rviz_common::ViewportMouseEvent & event);
  bool handleDisplayClick(
    rviz_common::properties::Property * property, rviz_common::ViewportMouseEvent & event);

  bool is_moving_;
  rviz_common::properties::Property * target_property_;
  std::string target_property_type_;
  int move_offset_x_, move_offset_y_;
  bool shift_pressing_;

private:
};
}  // namespace jsk_rviz_plugins

#endif
