cmake_minimum_required(VERSION 3.8)
project(depth_transport)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)   # [261017] cv_bridge 는 소스에서 쓰지 않아 뺐다 (sender 는 opencv2 를 직접 include)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED libzstd)

add_executable(depth_zstd_sender src/depth_zstd_sender.cpp)
ament_target_dependencies(depth_zstd_sender rclcpp sensor_msgs)
target_include_directories(depth_zstd_sender PRIVATE ${ZSTD_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_zstd_sender ${ZSTD_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depth_zstd_receiver src/depth_zstd_receiver.cpp)
ament_target_dependencies(depth_zstd_receiver rclcpp sensor_msgs)
target_include_directories(depth_zstd_receiver PRIVATE ${ZSTD_INCLUDE_DIRS})
target_link_libraries(depth_zstd_receiver ${ZSTD_LIBRARIES})

install(TARGETS depth_zstd_sender depth_zstd_receiver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
