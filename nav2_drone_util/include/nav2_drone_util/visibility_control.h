#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// Cross-platform visibility macros for symbol exporting/importing
// Reference: https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAV_DRONE_EXPORT __attribute__ ((dllexport)) // Export symbols for GCC on Windows
    #define NAV_DRONE_IMPORT __attribute__ ((dllimport)) // Import symbols for GCC on Windows
  #else
    #define NAV_DRONE_EXPORT __declspec(dllexport)       // Export symbols for MSVC on Windows
    #define NAV_DRONE_IMPORT __declspec(dllimport)       // Import symbols for MSVC on Windows
  #endif
  #ifdef NAV_DRONE_BUILDING_DLL
    #define NAV_DRONE_PUBLIC NAV_DRONE_EXPORT            // Public symbols for DLL build
  #else
    #define NAV_DRONE_PUBLIC NAV_DRONE_IMPORT            // Public symbols for DLL usage
  #endif
  #define NAV_DRONE_PUBLIC_TYPE NAV_DRONE_PUBLIC         // Public type alias
  #define NAV_DRONE_LOCAL                                // Local symbols not supported on Windows
#else
  #define NAV_DRONE_EXPORT __attribute__ ((visibility("default"))) // Default visibility for Unix
  #define NAV_DRONE_IMPORT                                         // Import is a no-op on Unix
  #if __GNUC__ >= 4
    #define NAV_DRONE_PUBLIC __attribute__ ((visibility("default"))) // Public symbols
    #define NAV_DRONE_LOCAL  __attribute__ ((visibility("hidden")))  // Hidden symbols
  #else
    #define NAV_DRONE_PUBLIC                                      // Public symbols (default visibility)
    #define NAV_DRONE_LOCAL                                       // Hidden symbols not supported
  #endif
  #define NAV_DRONE_PUBLIC_TYPE NAV_DRONE_PUBLIC                  // Public type alias
#endif

#ifdef __cplusplus
}
#endif
