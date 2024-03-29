cmake_minimum_required(VERSION 3.5)
project(camera_intrinsic_calibration)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(calibration_common REQUIRED)
find_package(calibration_interfaces REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  calibration_common
  calibration_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/pinhole_calibrator.cpp
  src/calibration_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
)

# calibration node
add_executable(calibration_node
  src/calibration_main.cpp)

target_link_libraries(calibration_node ${PROJECT_NAME})


# install include directories
install(DIRECTORY include/
  DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install executables
install(TARGETS
    calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

# install executable scripts
install(PROGRAMS
  scripts/calibration_client.py
  DESTINATION lib/${PROJECT_NAME})

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
