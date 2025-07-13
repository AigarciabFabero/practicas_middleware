#include <yarp/os/all.h>
#include <iostream>
#include <thread>

using namespace yarp::os;
using namespace std;

BufferedPort<Bottle> consumidorPort;

void productor(int id) {
    BufferedPort<Bottle> port;
    port.open("/productor" + to_string(id));
    Network::connect(port.getName(), "/consumidor");

    for (int i = 0; i < 10; i++) {
        Bottle& msg = port.prepare();
        msg.clear();
        msg.addString("Mensaje del Productor " + to_string(id) + " #" + to_string(i));
        port.write();
        Time::delay(0.5);
    }
}

void consumidor() {
    consumidorPort.open("/consumidor");
    while (true) {
        Bottle *msg = consumidorPort.read();
        if (msg) {
            cout << "Consumidor recibió: " << msg->toString() << endl;
        }
    }
}

int main() {
    Network yarp;
    if (!yarp.checkNetwork()) {
        cerr << "No se pudo conectar a YARP" << endl;
        return -1;
    }

    thread tConsumidor(consumidor);
    thread t1(productor, 1);
    thread t2(productor, 2);
    thread t3(productor, 3);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}