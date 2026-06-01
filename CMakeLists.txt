cmake_minimum_required(VERSION 3.5)
project(vector_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(library_name vector_pursuit_controller)

ament_auto_add_library(${library_name} SHARED
        src/vector_pursuit_controller.cpp)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(nav2_core vector_pursuit_controller.xml)

ament_auto_package()

