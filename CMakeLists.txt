cmake_minimum_required(VERSION 3.5)

project(ffmpeg_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# the image transport api changed between distros
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR
      $ENV{ROS_DISTRO} STREQUAL "galactic")
      add_definitions(-DIMAGE_TRANSPORT_API_V1)
    elseif($ENV{ROS_DISTRO} STREQUAL "humble")
      add_definitions(-DIMAGE_TRANSPORT_API_V2)
    else()
      add_definitions(-DIMAGE_TRANSPORT_API_V3)
    endif()
else()
  message(ERROR "ROS_DISTRO environment variable is not set!")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(ffmpeg_encoder_decoder REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


set(LIBRARY_NAME ${PROJECT_NAME}_component)

add_library(
  ${LIBRARY_NAME}
  SHARED
  src/ffmpeg_publisher.cpp
  src/ffmpeg_subscriber.cpp
  src/manifest.cpp)

target_include_directories(${LIBRARY_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(${LIBRARY_NAME}
PUBLIC
  image_transport
  pluginlib
  rclcpp
  sensor_msgs
  ffmpeg_encoder_decoder
  ffmpeg_image_transport_msgs)

ament_export_dependencies(
  image_transport
  pluginlib
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  ffmpeg_encoder_decoder
  ffmpeg_image_transport_msgs)

install(TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_targets(export_${LIBRARY_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(image_transport ffmpeg_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  if(NOT $ENV{ROS_DISTRO} STREQUAL "galactic")
    find_package(ament_cmake_pep257 REQUIRED)
  endif()

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_flake8()
  ament_lint_cmake()
  if(NOT $ENV{ROS_DISTRO} STREQUAL "galactic")
    ament_pep257()
  endif()
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_xmllint()
endif()

ament_package()

