cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(ViconDataStreamSDK_CPP
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP/DataStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/CoreClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/ClientUtils.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/AxisMapping.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClientSDK/CGClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamBayer.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/CGStreamReaderWriter.cpp
)
target_include_directories(ViconDataStreamSDK_CPP
  PUBLIC Vicon/CrossMarket/DataStream
  PRIVATE Vicon/CrossMarket)
target_link_libraries(ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
