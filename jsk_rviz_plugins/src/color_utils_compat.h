// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef JSK_RVIZ_PLUGINS_COLOR_UTILS_COMPAT_H_
#define JSK_RVIZ_PLUGINS_COLOR_UTILS_COMPAT_H_

// jsk_topic_tools::colorCategory20() and heatColor() of
// jsk_topic_tools/color_utils.h, which is ROS 1 only in jsk_topic_tools 3.x:
// the same functions for ROS 2, on std_msgs/msg/ColorRGBA.
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <algorithm>
#include <std_msgs/msg/color_rgba.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(std_msgs, ColorRGBA)

namespace jsk_topic_tools
{
  // choose a color out of the 20 colors of d3.js' category20
  inline std_msgs::ColorRGBA colorCategory20(int i)
  {
    static const double colors[20][3] = {
      {0.121569, 0.466667, 0.705882}, {0.682353, 0.780392, 0.909804},
      {1.000000, 0.498039, 0.054902}, {1.000000, 0.733333, 0.470588},
      {0.172549, 0.627451, 0.172549}, {0.596078, 0.874510, 0.541176},
      {0.839216, 0.152941, 0.156863}, {1.000000, 0.596078, 0.588235},
      {0.580392, 0.403922, 0.741176}, {0.772549, 0.690196, 0.835294},
      {0.549020, 0.337255, 0.294118}, {0.768627, 0.611765, 0.580392},
      {0.890196, 0.466667, 0.760784}, {0.968627, 0.713725, 0.823529},
      {0.498039, 0.498039, 0.498039}, {0.780392, 0.780392, 0.780392},
      {0.737255, 0.741176, 0.133333}, {0.858824, 0.858824, 0.552941},
      {0.090196, 0.745098, 0.811765}, {0.619608, 0.854902, 0.898039}};
    std_msgs::ColorRGBA c;
    c.r = colors[i % 20][0];
    c.g = colors[i % 20][1];
    c.b = colors[i % 20][2];
    c.a = 1.0;
    return c;
  }

  // heat-map color of v in [0.0:1.0]
  inline std_msgs::ColorRGBA heatColor(double v)
  {
    double ratio = 2 * v;
    int b = std::max(0.0, 255 * (1.0 - ratio));
    int r = std::max(0.0, 255 * (ratio - 1.0));
    int g = 255 - b - r;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
    return color;
  }
}  // namespace jsk_topic_tools
#else
#include <jsk_topic_tools/color_utils.h>
#endif

#endif  // JSK_RVIZ_PLUGINS_COLOR_UTILS_COMPAT_H_
