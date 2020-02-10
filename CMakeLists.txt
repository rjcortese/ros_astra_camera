cmake_minimum_required(VERSION 3.5)
project(astra_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

option(FILTER "OB Filter library" ON)

if( ${FILTER} STREQUAL "ON" )
set(obfilter "On")
else()
set(obfilter "Off")
endif()

include(ExternalProject)

if(WIN32)
  set(patch_exe patch.exe)
else()
  set(patch_exe patch)
endif()

ExternalProject_Add(astra_openni2
  PREFIX astra_openni2
  # SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/astra_openni2/OpenNI2
  GIT_REPOSITORY https://github.com/orbbec/OpenNI2.git
  GIT_TAG orbbec_ros
  GIT_CONFIG user.email=you@example.com user.name=YourName
  CONFIGURE_COMMAND echo "no need to configure"
  #${CMAKE_CURRENT_SOURCE_DIR}/libantlr/configure --prefix=<INSTALL_DIR>
  BUILD_IN_SOURCE 1
  BUILD_COMMAND make release FILTER=${obfilter}
  INSTALL_DIR openni2
  INSTALL_COMMAND tar -xjf <SOURCE_DIR>/Packaging/Final/OpenNI-Linux-2.3.tar.bz2 -C <INSTALL_DIR> --strip 1 && mkdir -p <INSTALL_DIR>/include && ln -fs <INSTALL_DIR>/Include <INSTALL_DIR>/include/openni2
  PATCH_COMMAND
  ${patch_exe} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/astra_openni2-patch.diff
)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2/Redist)
include_directories(include
  ${CMAKE_CURRENT_BINARY_DIR}/openni2/include
)


set(node_plugins "")


add_library(astra_driver_component SHARED
  src/astra_driver.cpp
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
)
target_compile_definitions(astra_driver_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(astra_driver_component
  "builtin_interfaces"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)
target_link_libraries(astra_driver_component
  -lOpenNI2Orbbec
)
add_dependencies(astra_driver_component
  astra_openni2
)
rclcpp_components_register_nodes(astra_driver_component
  "astra_wrapper::AstraDriver"
)
set(node_plugins
  "${node_plugins}astra_wrapper::AstraDriver;$<TARGET_FILE:astra_driver_component>\n"
)


if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


add_executable(astra_man_comp_alt
  ros/astra_man_comp_alt.cpp
)
target_link_libraries(astra_man_comp_alt
  astra_driver_component
)
ament_target_dependencies(astra_man_comp_alt
  "rclcpp"
)
  

install(TARGETS
  astra_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  astra_man_comp_alt
  DESTINATION lib/${PROJECT_NAME}
)


install(FILES ${CMAKE_BINARY_DIR}/openni2/Redist/libOpenNI2Orbbec.so
  DESTINATION lib
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/openni2/Redist/OpenNI2
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(FILES 56-orbbec-usb.rules
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_include_directories(include)
ament_package()
