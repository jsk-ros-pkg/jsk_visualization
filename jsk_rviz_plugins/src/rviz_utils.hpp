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

#ifndef JSK_RVIZ_PLUGINGS_RVIZ_UTIL_H_
#define JSK_RVIZ_PLUGINGS_RVIZ_UTIL_H_

#include <OgreColourValue.h>

#include <rviz_common/logging.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "string_utils.hpp"

#define JSK_LOG_DEBUG(x, ...)                                                          \
  do {                                                                                 \
    rviz_common::log_debug(rviz_string::format(x, ##__VA_ARGS__), __FILE__, __LINE__); \
  } while (0)

#define JSK_LOG_INFO(x, ...)                                                          \
  do {                                                                                \
    rviz_common::log_info(rviz_string::format(x, ##__VA_ARGS__), __FILE__, __LINE__); \
  } while (0)

#define JSK_LOG_WARN(x, ...)                                                             \
  do {                                                                                   \
    rviz_common::log_warning(rviz_string::format(x, ##__VA_ARGS__), __FILE__, __LINE__); \
  } while (0)

#define JSK_LOG_ERROR(x, ...)                                                          \
  do {                                                                                 \
    rviz_common::log_error(rviz_string::format(x, ##__VA_ARGS__), __FILE__, __LINE__); \
  } while (0)

namespace rviz
{
// inline Ogre::ColourValue colorMsgToOgre(std_msgs::msg::ColorRGBA::ConstSharedPtr c)
// {
//   return Ogre::ColourValue(c.r, c.g, c.b, c.a);
// }

// inline std_msgs::ColorRGBA colorOgreToMsg(const Ogre::ColourValue &c)
// {
//   std_msgs::ColorRGBA ret;
//   ret.r = c.r;
//   ret.g = c.g;
//   ret.b = c.b;
//   ret.a = c.a;
//   return ret;
// }

}  // namespace rviz

#endif  // JSK_RVIZ_PLUGINGS_RVIZ_UTIL_H_
