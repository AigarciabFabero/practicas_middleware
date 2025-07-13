// nodo_spinOnce.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

// Definimos el tamaño de la cola y la frecuencia del ciclo como constantes
const int QUEUE_SIZE = 10;
const double LOOP_RATE_HZ = 1.0; // 1 Hz (1 segundos)

// Callback para procesar los mensajes recibidos
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Nodo spinOnce(): He recibido: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // Inicialización del nodo
  ros::init(argc, argv, "nodo_spinOnce");
  ros::NodeHandle nh;

  // Suscripción al tópico "chatter" con el tamaño de cola definido
  ros::Subscriber sub = nh.subscribe("chatter", QUEUE_SIZE, chatterCallback);

  // Definición de la frecuencia del ciclo
  ros::Rate loop_rate(LOOP_RATE_HZ);

  ROS_INFO("Nodo spinOnce() iniciado. Ejecutando tareas adicionales...");

  // Bucle principal del nodo
  while (ros::ok())
  {
    // Procesa las callbacks disponibles
    ros::spinOnce();

    // Tarea adicional: imprimir un mensaje de estado
    ROS_INFO("Nodo spinOnce(): Ejecutando tareas adicionales.");

    // Controla la frecuencia del bucle
    loop_rate.sleep();
  }

  return 0;
}