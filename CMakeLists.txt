cmake_minimum_required(VERSION 3.8)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

include(requirements.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(FATAL_ERROR 
        "\n=======================================================================\n"
        "BUILD DISABLED ON ARM64:\n"
        "The upstream APT packages 'sumo-tools' and 'sumo-dev' are currently \n"
        "misconfigured for arm64, causing a build failure. \n"
        "This package is disabled on arm64 until those packages are fixed.\n"
        "======================================================================="
    )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

find_package(SUMO REQUIRED)

add_executable(${PROJECT} src/adore_sumo_bridge.cpp)

ament_target_dependencies(${PROJECT}
  rclcpp
  adore_ros2_msgs
)

target_link_libraries(${PROJECT}
  SUMO::libsumocpp
)

target_include_directories(${PROJECT} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  ${PROJECT}
  DESTINATION lib/${PROJECT}
)

ament_package()
