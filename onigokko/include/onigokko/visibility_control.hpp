#ifndef ONIGOKKO__VISIBILITY_CONTROL_HPP_
#define ONIGOKKO__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ONIGOKKO_EXPORT __attribute__ ((dllexport))
    #define ONIGOKKO_IMPORT __attribute__ ((dllimport))
  #else
    #define ONIGOKKO_EXPORT __declspec(dllexport)
    #define ONIGOKKO_IMPORT __declspec(dllimport)
  #endif
  #ifdef ONIGOKKO_BUILDING_LIBRARY
    #define ONIGOKKO_PUBLIC ONIGOKKO_EXPORT
  #else
    #define ONIGOKKO_PUBLIC ONIGOKKO_IMPORT
  #endif
  #define ONIGOKKO_PUBLIC_TYPE ONIGOKKO_PUBLIC
  #define ONIGOKKO_LOCAL
#else
  #define ONIGOKKO_EXPORT __attribute__ ((visibility("default")))
  #define ONIGOKKO_IMPORT
  #if __GNUC__ >= 4
    #define ONIGOKKO_PUBLIC __attribute__ ((visibility("default")))
    #define ONIGOKKO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ONIGOKKO_PUBLIC
    #define ONIGOKKO_LOCAL
  #endif
  #define ONIGOKKO_PUBLIC_TYPE
#endif

#endif
