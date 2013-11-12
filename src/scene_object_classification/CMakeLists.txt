cmake_minimum_required(VERSION 2.8.3)
project(scene_object_classification)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs strands_qsr_msgs)
find_package( PCL 1.4 REQUIRED COMPONENTS visualization filters segmentation )
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#   strands_qsr_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_object_classification
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs strands_qsr_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(strands_qsr_relational_estimator
#   src/${PROJECT_NAME}/strands_qsr_relational_estimator.cpp
# )
add_library( sceneClassification src/Object.cpp src/SceneInformation.cpp src/ApiConvertKTHDB.cpp src/DatabaseInformation.cpp  src/ApiFeatureExtraction.cpp src/ObjectFeatures.cpp src/ObjectPairFeatures.cpp src/FeatureInformation.cpp src/AllFeatSingleObject.cpp src/AllFeatPairObject.cpp src/TestScene.cpp src/LOOCV.cpp src/utils.cpp src/Training.cpp src/ApiConvertSimulationDB.cpp)

## Declare a cpp executable
add_executable(object_class_prediction_service src/object_class_prediction_service.cpp)
add_executable(test src/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_class_prediction_service strands_qsr_spatial_relation_estimator_generate_messages_cpp)
#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries( object_class_prediction_service ${catkin_LIBRARIES})
target_link_libraries( object_class_prediction_service sceneClassification)
target_link_libraries( test ${catkin_LIBRARIES})
target_link_libraries( test sceneClassification)
target_link_libraries( sceneClassification ${OpenCV_LIBS} )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS scene_object_classification scene_object_classification_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scene_object_classification.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
