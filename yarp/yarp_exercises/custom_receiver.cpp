#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <iostream>

using namespace yarp::os;

int main(int argc, char* argv[]) {
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);

    Network yarp;  // Inicializa YARP
    Bottle bot;    // Contenedor para los mensajes recibidos
    Port input;    // Puerto de entrada

    input.open("/receiver"); // Abre el puerto de lectura
    Network::connect("/custom_sender", "/receiver"); // Conexión automática

    while (true) {
        if (input.read(bot)) { // Leer mensaje
            std::cout << "Received message: " << bot.toString() << std::endl;
        }
    }

    input.close();  // Cierra el puerto al finalizar
    return 0;
}
