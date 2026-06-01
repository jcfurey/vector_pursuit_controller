# tests for VP — ament_auto_add_gtest pulls in <test_depend> + <depend> from package.xml
ament_auto_add_gtest(test_vp
  test_vector_pursuit.cpp
  path_utils/path_utils.cpp
)
target_link_libraries(test_vp
  ${library_name}
)

# Path utils test
ament_auto_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
