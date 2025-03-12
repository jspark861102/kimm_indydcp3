cmake_minimum_required(VERSION 3.10)
project(kimm_indydcp3)

# C++ standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build option
option(BUILD_PROTO "Build proto task" OFF)

# cmake include
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

# Find Protobuf and gRPC packages
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure 
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools  
  rospy
  roslib
  tf
  tf_conversions
  robotiq_2f_gripper_control
  robotiq_2f_gripper_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools    
    roscpp
    robotiq_2f_gripper_control
    robotiq_2f_gripper_msgs  
)

# Include directories for source files
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${gRPC_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/neuromeka_cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/neuromeka_cpp/proto)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

set(INDY_SOURCES
    neuromeka_cpp/indydcp3.cpp
    neuromeka_cpp/proto/common_msgs.pb.cc
    neuromeka_cpp/proto/config_msgs.pb.cc
    neuromeka_cpp/proto/config.pb.cc
    neuromeka_cpp/proto/control_msgs.pb.cc
    neuromeka_cpp/proto/control.pb.cc
    neuromeka_cpp/proto/device_msgs.pb.cc
    neuromeka_cpp/proto/device.pb.cc
    neuromeka_cpp/proto/ethercat_msgs.pb.cc
    neuromeka_cpp/proto/ethercat.pb.cc
    # neuromeka_cpp/proto/eyetask.pb.cc
    # neuromeka_cpp/proto/indyeye.pb.cc
    # neuromeka_cpp/proto/moby_msgs.pb.cc
    # neuromeka_cpp/proto/moby.pb.cc
    neuromeka_cpp/proto/rtde_msgs.pb.cc
    neuromeka_cpp/proto/rtde.pb.cc
    neuromeka_cpp/proto/cri.pb.cc
    neuromeka_cpp/proto/common_msgs.grpc.pb.cc
    neuromeka_cpp/proto/config_msgs.grpc.pb.cc
    neuromeka_cpp/proto/config.grpc.pb.cc
    neuromeka_cpp/proto/control_msgs.grpc.pb.cc
    neuromeka_cpp/proto/control.grpc.pb.cc
    neuromeka_cpp/proto/device_msgs.grpc.pb.cc
    neuromeka_cpp/proto/device.grpc.pb.cc
    neuromeka_cpp/proto/ethercat_msgs.grpc.pb.cc
    neuromeka_cpp/proto/ethercat.grpc.pb.cc
    # neuromeka_cpp/proto/eyetask.grpc.pb.cc
    # neuromeka_cpp/proto/indyeye.grpc.pb.cc
    # neuromeka_cpp/proto/moby_msgs.grpc.pb.cc
    # neuromeka_cpp/proto/moby.grpc.pb.cc
    neuromeka_cpp/proto/rtde_msgs.grpc.pb.cc
    neuromeka_cpp/proto/rtde.grpc.pb.cc
    neuromeka_cpp/proto/cri.grpc.pb.cc
)

add_executable(example_indydcp3 src/example_indydcp3.cpp ${INDY_SOURCES})
target_link_libraries(example_indydcp3 
                    ${Protobuf_LIBRARIES} 
                    ${gRPC_LIBRARIES}
                    ${_REFLECTION}
                    ${_GRPC_GRPCPP}
                    ${_PROTOBUF_LIBPROTOBUF}
                    ${catkin_LIBRARIES}
                    -lpthread
                    )

add_executable(kimm_example src/kimm_example.cpp ${INDY_SOURCES})
target_link_libraries(kimm_example 
                    ${Protobuf_LIBRARIES} 
                    ${gRPC_LIBRARIES}
                    ${_REFLECTION}
                    ${_GRPC_GRPCPP}
                    ${_PROTOBUF_LIBPROTOBUF}
                    ${catkin_LIBRARIES}
                    -lpthread
                    )


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
