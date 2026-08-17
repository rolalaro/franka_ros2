cmake_minimum_required(VERSION 3.8)
project(joint_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.19.0 REQUIRED)

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
ament_auto_find_build_dependencies()
find_package(pinocchio REQUIRED)

generate_parameter_library(joint_controllers_params
  src/joint_controllers_parameters.yaml
)

ament_auto_add_library(joint_controllers SHARED
  src/tsid_position_control.cpp
  src/joint_space_controller.cpp
  src/tsid_velocity_control.cpp
  src/joint_space_vel_controller.cpp
  src/interfaces.cpp
)

target_link_libraries(joint_controllers
                      joint_controllers_params
                      pinocchio::pinocchio  
                      Franka::Franka
)
target_include_directories(joint_controllers PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  pinocchio::pinocchio
  )


pluginlib_export_plugin_description_file(controller_interface joint_controllers.xml)

# if(BUILD_TESTING)

# endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE)
