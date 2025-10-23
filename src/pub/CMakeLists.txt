cmake_minimum_required(VERSION 3.5)
project(pub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 查找libudev库
find_package(PkgConfig REQUIRED)
pkg_check_modules(udev REQUIRED libudev)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(include)

add_executable(pub src/cam.cpp src/pub.cpp)

ament_target_dependencies(pub
rclcpp
udev
sensor_msgs
cv_bridge
OpenCV
)

target_link_libraries(pub  
  ${PROJECT_SOURCE_DIR}/lib/libtstc_usbcam.so
  ${udev_LIBRARIES}
)

install(TARGETS
pub
DESTINATION lib/${PROJECT_NAME}
)

# 安装第三方动态库
install(DIRECTORY lib/  
  DESTINATION lib/     
)
