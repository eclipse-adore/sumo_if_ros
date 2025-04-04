cmake_minimum_required(VERSION 3.8)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

include($ENV{SOURCE_DIRECTORY}/libraries/adore_libraries.cmake)

include(requirements.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

# Ensure the Release build uses -O3 for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()




add_executable(${PROJECT} src/adore_sumo_bridge.cpp)
target_link_libraries(${PROJECT} PUBLIC ${SUMOCPP_LIBRARY} gdal)

target_include_directories(${PROJECT} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/sumo/sumo/src/>
    $<INSTALL_INTERFACE:include>
)

add_all_target_include_directories(${PROJECT})
add_all_target_link_libraries(${PROJECT})

install(TARGETS
    ${PROJECT}
  DESTINATION lib/${PROJECT}
)


install(FILES ${SUMOCPP_LIBRARY}
        DESTINATION lib)

        
ament_package()
