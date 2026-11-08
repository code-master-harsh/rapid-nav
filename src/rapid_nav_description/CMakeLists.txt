cmake_minimum_required(VERSION 3.8)
project(rapid_nav_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Only the XML lint and the Python lint required for every package; no copyright or cppcheck.
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 240
  )
endif()

ament_package()
