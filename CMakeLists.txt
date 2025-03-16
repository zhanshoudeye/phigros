cmake_minimum_required(VERSION 3.8)
project(phigros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

include_directories(include/phigros)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
        src/image_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
