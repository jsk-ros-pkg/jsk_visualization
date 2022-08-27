#ifndef _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_
#define _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_RVIZ_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define JSK_RVIZ_PLUGINS_IMPORT __attribute__ ((dllimport))
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
  #define JSK_RVIZ_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define JSK_RVIZ_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define JSK_RVIZ_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_RVIZ_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_RVIZ_PLUGINS_PUBLIC
    #define JSK_RVIZ_PLUGINS_LOCAL
  #endif
  #define JSK_RVIZ_PLUGINS_PUBLIC_TYPE
#endif

#endif  // _JSK_RVIZ_PLUGINS_VISIBILITY_CONTROL_HPP_