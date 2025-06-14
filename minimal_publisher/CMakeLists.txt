cmake_minimum_required(VERSION 3.14)
project(minpub)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


file(GLOB SRC_STANDSIT
  ${PROJECT_SOURCE_DIR}/*.cpp
)

add_executable(minpub
  ${SRC_STANDSIT}
)

ament_target_dependencies(minpub
  ${TARGET_DEPENDENCIES}
)

install(TARGETS
  minpub
)

# ament_package()
