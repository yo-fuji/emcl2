#ifndef EMCL2__VISIBILITY_CONTROL_H_
#define EMCL2__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define EMCL2_EXPORT __attribute__((dllexport))
#define EMCL2_IMPORT __attribute__((dllimport))
#else
#define EMCL2_EXPORT __declspec(dllexport)
#define EMCL2_IMPORT __declspec(dllimport)
#endif
#ifdef EMCL2_BUILDING_LIBRARY
#define EMCL2_PUBLIC EMCL2_EXPORT
#else
#define EMCL2_PUBLIC EMCL2_IMPORT
#endif
#define EMCL2_PUBLIC_TYPE EMCL2_PUBLIC
#define EMCL2_LOCAL
#else
#define EMCL2_EXPORT __attribute__((visibility("default")))
#define EMCL2_IMPORT
#if __GNUC__ >= 4
#define EMCL2_PUBLIC __attribute__((visibility("default")))
#define EMCL2_LOCAL __attribute__((visibility("hidden")))
#else
#define EMCL2_PUBLIC
#define EMCL2_LOCAL
#endif
#define EMCL2_PUBLIC_TYPE
#endif

#endif // EMCL2__VISIBILITY_CONTROL_H_
