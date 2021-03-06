cmake_minimum_required(VERSION 2.8.3)
project(plane_calibration)

find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_geometry
  cmake_modules
  roscpp
  nodelet
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
  tf2
  tf2_msgs
  image_geometry
  depth_image_proc
  eigen_conversions
  tf2_ros
)

ecl_enable_cxx11_compiler()

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/PlaneCalibration.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        ecl_build
        ecl_geometry
        roscpp
        nodelet
        dynamic_reconfigure
        sensor_msgs
        geometry_msgs
        tf2
        tf2_msgs
        image_geometry
        depth_image_proc
        eigen_conversions
        tf2_ros
    DEPENDS Eigen
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

add_subdirectory(src)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})