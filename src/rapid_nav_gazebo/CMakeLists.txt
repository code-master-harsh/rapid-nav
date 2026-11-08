cmake_minimum_required(VERSION 3.8)
project(rapid_nav_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Python lint for every package (flake8, pep257); no copyright or cppcheck.
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_world test/test_world.py
    TIMEOUT 240
  )

  ament_add_pytest_test(test_launch test/test_launch.py
    TIMEOUT 120
  )
endif()

ament_package()
