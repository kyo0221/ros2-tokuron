#ifndef TELEOP_OMNI__VISIBILITY_CONTROL_HPP_
#define TELEOP_OMNI__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TELEOP_OMNI_EXPORT __attribute__ ((dllexport))
    #define TELEOP_OMNI_IMPORT __attribute__ ((dllimport))
  #else
    #define TELEOP_OMNI_EXPORT __declspec(dllexport)
    #define TELEOP_OMNI_IMPORT __declspec(dllimport)
  #endif
  #ifdef TELEOP_OMNI_BUILDING_LIBRARY
    #define TELEOP_OMNI_PUBLIC TELEOP_OMNI_EXPORT
  #else
    #define TELEOP_OMNI_PUBLIC TELEOP_OMNI_IMPORT
  #endif
  #define TELEOP_OMNI_PUBLIC_TYPE TELEOP_OMNI_PUBLIC
  #define TELEOP_OMNI_LOCAL
#else
  #define TELEOP_OMNI_EXPORT __attribute__ ((visibility("default")))
  #define TELEOP_OMNI_IMPORT
  #if __GNUC__ >= 4
    #define TELEOP_OMNI_PUBLIC __attribute__ ((visibility("default")))
    #define TELEOP_OMNI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TELEOP_OMNI_PUBLIC
    #define TELEOP_OMNI_LOCAL
  #endif
  #define TELEOP_OMNI_PUBLIC_TYPE
#endif

#endif
