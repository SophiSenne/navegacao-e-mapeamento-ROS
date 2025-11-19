cmake_minimum_required(VERSION 3.8)
project(navegacao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(grafo_node 
  src/main.cpp
  src/ControladorRobo.cpp
  src/Algoritmo.cpp
  src/Grafo.cpp
  src/ExploradorLabirinto.cpp
)

target_include_directories(grafo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(grafo_node PUBLIC cxx_std_17)
ament_target_dependencies(grafo_node
  rclcpp
  cg_interfaces
)

install(TARGETS 
  grafo_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()