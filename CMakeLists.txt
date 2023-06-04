cmake_minimum_required(VERSION 3.8)
project(emcl2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(Pose_emcl2 SHARED src/Pose.cpp)
ament_auto_add_library(Scan_emcl2 SHARED src/Scan.cpp)
ament_auto_add_library(LikelihoodFieldMap_emcl2 SHARED src/LikelihoodFieldMap.cpp)
ament_auto_add_library(OdomModel_emcl2 SHARED src/OdomModel.cpp)
ament_auto_add_library(Particle_emcl2 SHARED src/Particle.cpp)
ament_auto_add_library(Mcl_emcl2 SHARED src/Mcl.cpp)
ament_auto_add_library(ExpResetMcl2_emcl2 SHARED src/ExpResetMcl2.cpp)
ament_auto_add_library(emcl2 SHARED src/emcl2_node.cpp)

rclcpp_components_register_node(emcl2
  PLUGIN "emcl2::EMcl2Node"
  EXECUTABLE emcl2_node)
target_compile_features(emcl2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(emcl2 PRIVATE "EMCL2_BUILDING_LIBRARY")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config)
