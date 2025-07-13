#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <iostream>

using yarp::os::Bottle;
using yarp::os::Network;
using yarp::os::Port;

constexpr double loop_delay = 1.0; // Tiempo entre envíos
constexpr size_t top = 50; // Número total de iteraciones

int main(int argc, char* argv[]) {
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);

    Network yarp;  // Inicializa la red de YARP
    Port output;   // Crea el puerto de escritura

    output.open("/custom_sender");  // Abre el puerto de escritura

    for (size_t i = 1; i <= top; i++) {
        Bottle bot;
        bot.addString("Message");
        bot.addInt32(i);
        bot.addString("out of");
        bot.addInt32(top);
        bot.addString("sent by /custom_sender");

        output.write(bot);  // Envía el mensaje
        std::cout << "Sent message: " << bot.toString() << std::endl;

        yarp::os::Time::delay(loop_delay); // Espera un momento

        if (i == top / 2) {
            Bottle midBot;
            midBot.addString("Halfway through the sequence!");
            output.write(midBot);
            std::cout << "Sent message: " << midBot.toString() << std::endl;
        }
    }

    // Mensaje final
    Bottle endBot;
    endBot.addString("Sequence complete! Total messages sent:");
    endBot.addInt32(top);
    output.write(endBot);
    std::cout << "Sent message: " << endBot.toString() << std::endl;

    output.close();  // Cierra el puerto de escritura
    return 0;
}
