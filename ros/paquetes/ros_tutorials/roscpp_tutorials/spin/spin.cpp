// nodo_spin.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

// Definimos el tamaño de la cola como una constante
const int QUEUE_SIZE = 10;

// Callback para procesar los mensajes recibidos
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Nodo spin(): He recibido: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // Inicialización del nodo
  ros::init(argc, argv, "nodo_spin");
  ros::NodeHandle nh;

  // Suscripción al tópico "chatter" con el tamaño de cola definido
  ros::Subscriber sub = nh.subscribe("chatter", QUEUE_SIZE, chatterCallback);
  ROS_INFO("Nodo spin() iniciado. Esperando mensajes...");

  // Mantiene el nodo en ejecución, manejando callbacks indefinidamente
  ros::spin();

  return 0;
}