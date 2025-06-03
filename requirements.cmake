find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)




list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/build/sumo/bin")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/build/sumo/bin")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/build/sumo/build")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/build/sumo/build")

find_library(SUMOCPP_LIBRARY
             NAMES sumocpp
             PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/build/sumo/bin
             REQUIRED)


