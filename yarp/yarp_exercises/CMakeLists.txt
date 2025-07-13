cmake_minimum_required(VERSION 3.5)

# Nombre del proyecto
project(YARPExercises)

# Buscar YARP
find_package(YARP REQUIRED)

# Incluir cabeceras de YARP
include_directories(${YARP_INCLUDE_DIRS})

# Anyadir ejemplos
add_executable(sender sender.cpp)
add_executable(receiver receiver.cpp)
add_executable(custom_sender custom_sender.cpp)
add_executable(custom_receiver custom_receiver.cpp)
add_executable(custom_auto_connect custom_auto_connect.cpp)
add_executable(buffered_port buffered_port.cpp)
add_executable(custom_buffered_port custom_buffered_port.cpp)
add_executable(multithread multithread.cpp)
add_executable(multihilo multihilo.cpp)

# Vincular los ejecutables con las bibliotecas de YARP
target_link_libraries(sender ${YARP_LIBRARIES})
target_link_libraries(receiver ${YARP_LIBRARIES})
target_link_libraries(custom_sender ${YARP_LIBRARIES})
target_link_libraries(custom_receiver ${YARP_LIBRARIES})
target_link_libraries(custom_auto_connect ${YARP_LIBRARIES})
target_link_libraries(buffered_port ${YARP_LIBRARIES})
target_link_libraries(custom_buffered_port ${YARP_LIBRARIES})
target_link_libraries(multithread ${YARP_LIBRARIES})
target_link_libraries(multihilo ${YARP_LIBRARIES})