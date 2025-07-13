cmake_minimum_required(VERSION 3.0.2)
project(example_pkg)

## Habilitar C++11
add_compile_options(-std=c++11)

## Encontrar paquetes necesarios
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Declarar el paquete catkin
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

## Incluir directorios
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Agregar el ejecutable del nodo
add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES})
