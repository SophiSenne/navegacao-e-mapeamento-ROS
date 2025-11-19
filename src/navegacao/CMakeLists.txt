cmake_minimum_required(VERSION 3.8)
project(navegacao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Executável talker
add_executable(talker src/talker.cpp)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(talker
  "rclcpp"
  "std_msgs"
  "cg_interfaces"
)

# Executável grafo_node
add_executable(grafo_node 
  src/main.cpp
  src/Grafo.cpp
)
# NÃO inclua Grafo.h aqui, apenas os .cpp

target_include_directories(grafo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(grafo_node PUBLIC cxx_std_17)
ament_target_dependencies(grafo_node
  rclcpp
  cg_interfaces
)

# INSTALAR AMBOS OS EXECUTÁVEIS
install(TARGETS 
  talker
  grafo_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()