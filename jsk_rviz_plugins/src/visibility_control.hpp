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
#ifndef _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_
#define _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JSK_RVIZ_PLUGINS_EXPORT __attribute__((dllexport))
#define JSK_RVIZ_PLUGINS_IMPORT __attribute__((dllimport))
#else
#define JSK_RVIZ_PLUGINS_EXPORT __declspec(dllexport)
#define JSK_RVIZ_PLUGINS_IMPORT __declspec(dllimport)
#endif
#ifdef JSK_RVIZ_PLUGINS_BUILDING_LIBRARY
#define JSK_RVIZ_PLUGINS_PUBLIC JSK_RVIZ_PLUGINS_EXPORT
#else
#define JSK_RVIZ_PLUGINS_PUBLIC JSK_RVIZ_PLUGINS_IMPORT
#endif
#define JSK_RVIZ_PLUGINS_PUBLIC_TYPE JSK_RVIZ_PLUGINS_PUBLIC
#define JSK_RVIZ_PLUGINS_LOCAL
#else
#define JSK_RVIZ_PLUGINS_EXPORT __attribute__((visibility("default")))
#define JSK_RVIZ_PLUGINS_IMPORT
#if __GNUC__ >= 4
#define JSK_RVIZ_PLUGINS_PUBLIC __attribute__((visibility("default")))
#define JSK_RVIZ_PLUGINS_LOCAL __attribute__((visibility("hidden")))
#else
#define JSK_RVIZ_PLUGINS_PUBLIC
#define JSK_RVIZ_PLUGINS_LOCAL
#endif
#define JSK_RVIZ_PLUGINS_PUBLIC_TYPE
#endif

#endif  // _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_
