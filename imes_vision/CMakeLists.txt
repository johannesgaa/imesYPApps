cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_executable(vision_template example/vision_template.cpp)
#rosbuild_add_executable(colorCalibration_node example/colorCalibration_node.cpp)

#################### uncomment to compile the original imes_vision  ###################################
#rosbuild_genmsg()
#rosbuild_gensrv()
#rosbuild_add_library(imes_vision src/feature_matcher.cpp src/shape_matcher.cpp src/ObjectFinder.cpp)
#rosbuild_add_executable(ObjectFinder src/ObjectFinder_node.cpp)
#rosbuild_add_executable(ShapeDetector_service src/Shape_Detector_Service_node.cpp)
#rosbuild_add_executable(BottleDetector_topic src/Bottle_Detector_Topic_node.cpp)
#rosbuild_add_executable(FeatureDetector_service src/Feature_Detector_Service_node.cpp)
#target_link_libraries(ObjectFinder imes_vision)
#target_link_libraries(FeatureDetector_service imes_vision)
#target_link_libraries(ShapeDetector_service imes_vision)
#target_link_libraries(BottleDetector_topic imes_vision)

################### TestFinder!! ####################################
#rosbuild_genmsg()
#rosbuild_add_library(imes_vision_TEST src/feature_matcher.cpp src/shape_matcher.cpp src/TestFinder.cpp)
#rosbuild_add_executable(TestFinder src/TestFinder.cpp)
#target_link_libraries(TestFinder imes_vision_TEST)

################### algoTest!! ####################################
#rosbuild_add_library(imes_vision src/feature_matcher.cpp src/shape_matcher.cpp src/algoTest.cpp)
#rosbuild_add_executable(algoTest src/algoTest.cpp)
#target_link_libraries(algoTest imes_vision)
#rosbuild_link_boost(algoTest thread)

################### RoboCup!! ####################################
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()
#rosbuild_add_library(imes_vision_RC src/feature_matcher.cpp src/shape_matcher.cpp src/RoboCup2012.cpp)
#rosbuild_add_executable(RC_vision src/RoboCup2012.cpp)
#target_link_libraries(RC_vision imes_vision_RC)
#rosbuild_link_boost(RC_vision thread)

################### FeatureFinder!! ####################################
rosbuild_gensrv()
rosbuild_genmsg()
rosbuild_add_library(imes_vision src/feature_matcher.cpp src/RC_feature.cpp)
rosbuild_add_executable(FeatureFinder src/RC_feature.cpp)
target_link_libraries(FeatureFinder imes_vision)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
rosbuild_link_boost(FeatureFinder thread)
