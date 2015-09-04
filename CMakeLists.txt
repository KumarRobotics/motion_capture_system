cmake_minimum_required(VERSION 2.8.3)
project(motion_capture_system)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  tf
  eigen_conversions
  tf_conversions
)

if(NOT APPLE)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/vicon/vicon_sdk/Linux64")
    set(VICON_BOOST_LIB_SUFFIX ".so.1.51.0")
    list(APPEND CMAKE_FIND_LIBRARY_SUFFIXES ${VICON_BOOST_LIB_SUFFIX})
    find_library(VICON_BOOST_THREAD_LIB boost_thread-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB boost_system-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB boost_chrono-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    list(REMOVE_ITEM CMAKE_FIND_LIBRARY_SUFFIXES ${VICON_BOOST_LIB_SUFFIX})
    unset(VICON_BOOST_LIB_SUFFIX)
    set(VICON_BOOST_LIBS ${VICON_BOOST_THREAD_LIB} ${VICON_BOOST_SYSTEM_LIB}
      ${VICON_BOOST_CHRONO_LIB})
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(SDK_DIR "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/vicon/vicon_sdk/Linux32")
    set(VICON_BOOST_LIB_SUFFIX ".so.1.51.0")
    list(APPEND CMAKE_FIND_LIBRARY_SUFFIXES ${VICON_BOOST_LIB_SUFFIX})
    find_library(VICON_BOOST_WAVE_LIB boost_wave-mt ${SDK_DIR} NO_DEFAULT_PATH)
    find_library(VICON_BOOST_THREAD_LIB boost_thread-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SYSTEM_LIB boost_system-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_SERIALIZATION_LIB boost_serialization-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_DATE_TIME_LIB boost_date_time-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_CHRONO_LIB boost_chrono-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    find_library(VICON_BOOST_FILESYSTEM_LIB boost_filesystem-mt ${SDK_DIR}
      NO_DEFAULT_PATH)
    unset(VICON_BOOST_LIB_SUFFIX)
    set(VICON_BOOST_LIBS ${VICON_BOOST_WAVE_LIB} ${VICON_BOOST_THREAD_LIB}
      ${VICON_BOOST_SYSTEM_LIB} ${VICON_BOOST_SERIALIZATION_LIB}
      ${VICON_BOOST_DATE_TIME_LIB} ${VICON_BOOST_CHRONO_LIB}
      ${VICON_BOOST_FILESYSTEM_LIB})
  endif()
  #message(STATUS "SDK_DIR: ${SDK_DIR}")
  #message(STATUS "VICON_BOOST_LIBS: ${VICON_BOOST_LIBS}")
else(NOT APPLE)
  set(SDK_DIR "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/vicon/vicon_sdk/MacOSX")
endif()

find_library(VICON_SDK_LIB ViconDataStreamSDK_CPP ${SDK_DIR} NO_DEFAULT_PATH)
find_library(VICON_DEBUG_SERVICES_LIB DebugServices ${SDK_DIR} NO_DEFAULT_PATH)

#add_message_files(DIRECTORY msg)
#generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime geometry_msgs tf
  DEPENDS
  Boost Eigen3
)

include_directories(
  include
  ${SDK_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions("-std=c++0x -Wall")

add_library(mocap_kalman_filter
  src/KalmanFilter.cpp
)
target_link_libraries(mocap_kalman_filter
  ${catkin_LIBRARIES}
)
add_dependencies(mocap_kalman_filter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# VICON
add_library(mocap_vicon_driver
  src/MoCapDriverBase.cpp
  src/vicon/ViconDriver.cpp
)
target_link_libraries(mocap_vicon_driver
  ${VICON_SDK_LIB}
  ${VICON_DEBUG_SERVICES_LIB}
  ${VICON_BOOST_LIBS}
  mocap_kalman_filter
  ${catkin_LIBRARIES}
)
add_dependencies(mocap_vicon_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(mocap_vicon_node
  src/vicon/vicon.cpp
)
target_link_libraries(mocap_vicon_node
  mocap_vicon_driver
  ${catkin_LIBRARIES}
)

# QUALISYS
add_library(mocap_qualisys_driver
  src/MoCapDriverBase.cpp
  src/qualisys/QualisysDriver.cpp
  include/motion_capture_system/qualisys/RTProtocol.cpp
  include/motion_capture_system/qualisys/NBC_Markup.cpp
  include/motion_capture_system/qualisys/RTPacket.cpp
  include/motion_capture_system/qualisys/Network.cpp
)
target_link_libraries(mocap_qualisys_driver
  mocap_kalman_filter
  ${catkin_LIBRARIES}
)
add_dependencies(mocap_qualisys_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}

  ${catkin_EXPORTED_TARGETS}
)

add_executable(mocap_qualisys_node
  src/qualisys/qualisys.cpp
)
target_link_libraries(mocap_qualisys_node
  mocap_qualisys_driver
  ${catkin_LIBRARIES}
)

