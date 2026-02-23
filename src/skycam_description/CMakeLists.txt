cmake_minimum_required(VERSION 3.8)
project(skycam_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# NOTA: No buscamos rclpy aqui porque es Python. CMake solo gestiona C++.

# Instalación del script de Python
install(PROGRAMS
  scripts/simple_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalación de carpetas de recursos
install(DIRECTORY 
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()