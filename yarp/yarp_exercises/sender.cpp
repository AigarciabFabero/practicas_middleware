// Sender
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <iostream>

using namespace yarp::os;

int main() {
    Network yarp;
    BufferedPort<Bottle> senderPort;
    senderPort.open("/sender");

    int count = 0;
    while (true) {
        Bottle& message = senderPort.prepare();
        message.clear();
        message.addString("Message from sender");
        message.addInt32(++count);
        senderPort.write();
        std::cout << "Sender sent: " << message.toString() << std::endl;
        Time::delay(0.2); // Envia mensajes cada 200 ms
    }

    senderPort.close();
    return 0;
}