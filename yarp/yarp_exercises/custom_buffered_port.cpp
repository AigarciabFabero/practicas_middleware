#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <iostream>
#include <thread>

using namespace yarp::os;

int main() {
    Network yarp;
    
    BufferedPort<Bottle> producer1Port;
    BufferedPort<Bottle> producer2Port;
    BufferedPort<Bottle> consumerPort;
    
    producer1Port.open("/producer1");
    producer2Port.open("/producer2");
    consumerPort.open("/consumer");
    
    Network::connect("/producer1", "/consumer");
    Network::connect("/producer2", "/consumer");
    
    std::thread producer1([&]() {
        while (true) {
            Bottle& msg = producer1Port.prepare();
            msg.clear();
            msg.addString("Hello from Producer 1");
            producer1Port.write();
            std::cout << "Producer 1 sent: Hello from Producer 1" << std::endl;
            Time::delay(2.0);
        }
    });
    
    std::thread producer2([&]() {
        while (true) {
            Bottle& msg = producer2Port.prepare();
            msg.clear();
            msg.addString("Hello from Producer 2");
            producer2Port.write();
            std::cout << "Producer 2 sent: Hello from Producer 2" << std::endl;
            Time::delay(1.0);
        }
    });
    
    while (true) {
        Bottle* receivedMessage = consumerPort.read();
        if (receivedMessage) {
            std::cout << "Consumer received: " << receivedMessage->toString() << std::endl;
        }
    }
    
    producer1Port.close();
    producer2Port.close();
    consumerPort.close();
    
    producer1.join();
    producer2.join();
    
    return 0;
}
