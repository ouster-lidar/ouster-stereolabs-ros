cmake_minimum_required(VERSION 3.8)
project(ouster_zed)

# silence dev warnings raised by PCL's own find modules (PCLConfig resets
# policies internally, so set the defaults rather than the policies)
foreach(policy CMP0074 CMP0144)
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
    set(CMAKE_POLICY_DEFAULT_${policy} NEW)
  endif()
endforeach()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# FPFH/SAC-IA are an order of magnitude slower without optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features kdtree search registration)
find_package(OpenMP)

# Global registration node, built as a component (loadable into any container)
# and also as a standalone executable.
add_library(global_registration_component SHARED src/global_registration_node.cpp)
target_include_directories(global_registration_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(global_registration_component SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(global_registration_component ${PCL_LIBRARIES} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(global_registration_component OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(global_registration_component
  rclcpp rclcpp_components sensor_msgs message_filters tf2 tf2_ros tf2_eigen pcl_conversions)

rclcpp_components_register_node(global_registration_component
  PLUGIN "ouster_zed::GlobalRegistrationNode"
  EXECUTABLE global_registration_node)

install(TARGETS global_registration_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
