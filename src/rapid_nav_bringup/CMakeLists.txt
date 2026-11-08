cmake_minimum_required(VERSION 3.8)
project(rapid_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Python lint for every package (flake8, pep257); no copyright or cppcheck.
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_bringup test/test_bringup.py
    TIMEOUT 120
  )
endif()

ament_package()
