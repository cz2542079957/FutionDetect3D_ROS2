cmake_minimum_required(VERSION 3.8)
project(fusionDetect3D)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(std_srvs REQUIRED)     
find_package(message REQUIRED)    
# 查找udev
find_library(UDEV_LIBRARY NAMES udev)
find_path(UDEV_INCLUDE_DIR NAMES libudev.h) 

# 雷达sdk
set(SLLIDAR_SDK_PATH "./lib/lidarSdk") 
FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)
# 串口包
set(SERIAL_LIB_PATH "./lib/serial")
FILE(GLOB SERIAL_LIB_SRC
  "${SERIAL_LIB_PATH}/src/*.cc" 
  "${SERIAL_LIB_PATH}/src/list_ports/*.cc"
)
# 串口包
set(MSG_LIB_PATH "./lib/message")
FILE(GLOB MSG_LIB_SRC
  "${MSG_LIB_PATH}/msg/*.hpp" 
  "${MSG_LIB_PATH}/msg/detail/*.hpp"
)

include_directories(
  ${SLLIDAR_SDK_PATH}/include 
  ${SLLIDAR_SDK_PATH}/src 
  ${SERIAL_LIB_PATH}/include 
  ${SERIAL_LIB_PATH}/src
  ${MSG_LIB_PATH}/msg
  ${Boost_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIR}
  ./src
  ./src/lidar
  ./src/imu
  ./lib
  ./include
  ./include/lidar
  ./include/imu
)
 
# 项目文件
FILE(GLOB SOURCES
  "./include/lidar/*.h"
  "./include/imu/*.h"
  "./include/*.h"
  ./main.cpp
  "./src/lidar.*.cpp"
  "./src/imu/*.cpp"
  "./src/*.cpp"
)

add_executable(main ${SOURCES} ${SLLIDAR_SDK_SRC} ${SERIAL_LIB_SRC} ${MSG_LIB_SRC}) 
ament_target_dependencies(
  main
  rclcpp
  std_msgs 
  sensor_msgs
  std_srvs 
  message
  # imuData
)

  
target_link_libraries(main ${UDEV_LIBRARY})

install(TARGETS main DESTINATION lib/${PROJECT_NAME})
target_compile_options(main PRIVATE "-w")  # 禁用警告
ament_package()
