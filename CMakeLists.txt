cmake_minimum_required(VERSION 3.8)
project(fusionDetect3D)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(SLLIDAR_SDK_PATH "./lib/lidarSdk") 
FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)
set(SERIAL_LIB_PATH "./lib/serial")
FILE(GLOB SERIAL_LIB_SRC
  "${SERIAL_LIB_PATH}/src/*.cc" 
  "${SERIAL_LIB_PATH}/src/list_ports/*.cc"
)

include_directories(
  ${SLLIDAR_SDK_PATH}/include 
  ${SLLIDAR_SDK_PATH}/src 
  ${SERIAL_LIB_PATH}/include 
  ${SERIAL_LIB_PATH}/src
  ${Boost_INCLUDE_DIRS}
  ./src
  ./src/lidar
  ./include
  ./include/lidar
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(std_srvs REQUIRED) 

 
set(SOURCES 
  ./include/lidar/lidarNode.h
  ./include/lidar/lidarConfig.h

  ./main.cpp
  ./src/lidar/lidarNode.cpp
  ./src/lidar/lidarConfig.cpp
) 

add_executable(main ${SOURCES} ${SLLIDAR_SDK_SRC} ${SERIAL_LIB_SRC})

ament_target_dependencies(
  main
  rclcpp
  std_msgs 
  sensor_msgs
  std_srvs
)

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
target_compile_options(main PRIVATE "-w")  # 禁用警告
ament_package()
