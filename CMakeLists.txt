cmake_minimum_required(VERSION 3.8)
project(fusion_detect_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") 


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)   
find_package(message REQUIRED) 

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
 


include_directories(
  ${SLLIDAR_SDK_PATH}/include 
  ${SLLIDAR_SDK_PATH}/src 
  ${SERIAL_LIB_PATH}/include 
  ${SERIAL_LIB_PATH}/src 
  ${Boost_INCLUDE_DIRS} 
  ./src
  ./src/nodes 
  ./lib
  ./include
  ./include/nodes 
)
 
# 项目文件
FILE(GLOB SOURCES 
  "./include/nodes/*.h"
  "./include/*.h"
  ./main.cpp 
  "./src/nodes/*.cpp"
  "./src/*.cpp"
)

# 创建可执行文件并链接所需依赖
add_executable(main ${SOURCES} ${SLLIDAR_SDK_SRC} ${SERIAL_LIB_SRC} ) 
ament_target_dependencies(
  main
  rclcpp
  std_msgs  
  message  
)

# 安装目标到指定位置
install(TARGETS main DESTINATION lib/${PROJECT_NAME})
target_compile_options(main PRIVATE "-w")  # 禁用警告
ament_package()
