#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>
#include <iostream>

using namespace yarp::os;

class SenderThread : public Thread {
private:
    BufferedPort<Bottle> senderPort;
    bool running;

public:
    SenderThread() : running(true) {}

    bool threadInit() override {
        if (!senderPort.open("/sender")) {
            std::cerr << "Error: Could not open sender port" << std::endl;
            return false;
        }
        return true;
    }

    void run() override {
        int count = 0;
        while (!isStopping()) {
            Bottle& message = senderPort.prepare();
            message.clear();
            message.addString("Message from sender");
            message.addInt32(++count);
            senderPort.write();
            std::cout << "Sender sent: " << message.toString() << std::endl;
            Time::delay(1.0);
        }
    }

    void threadRelease() override {
        senderPort.close();
    }

    void stop() {
        running = false;
        Thread::stop();
    }
};

class ReceiverThread : public Thread {
private:
    BufferedPort<Bottle> receiverPort;

public:
    ReceiverThread() {}

    bool threadInit() override {
        if (!receiverPort.open("/receiver")) {
            std::cerr << "Error: Could not open receiver port" << std::endl;
            return false;
        }
        return true;
    }

    void run() override {
        while (!isStopping()) {
            Bottle* receivedMessage = receiverPort.read(); 
            if (receivedMessage != nullptr) {
                std::cout << "Receiver received: " << receivedMessage->toString() << std::endl;
            } else {
                Time::delay(0.1); 
            }
        }
    }

    void threadRelease() override {
        receiverPort.close();
    }

    void stop() {
        Thread::stop();
    }
};

int main() {
    Network yarp;

    if (!yarp.checkNetwork()) {
        std::cerr << "Error: YARP network is not available" << std::endl;
        return -1;
    }

    SenderThread senderThread;
    ReceiverThread receiverThread;

    if (!senderThread.start()) {
        std::cerr << "Error: Could not start sender thread" << std::endl;
        return -1;
    }

    if (!receiverThread.start()) {
        std::cerr << "Error: Could not start receiver thread" << std::endl;
        return -1;
    }

    if (!Network::connect("/sender", "/receiver")) {
        std::cerr << "Error: Could not connect sender to receiver" << std::endl;

        senderThread.stop();
        receiverThread.stop();
        senderThread.join();
        receiverThread.join();
        return -1;
    }

    Time::delay(10.0); 

    senderThread.stop();
    receiverThread.stop();

    senderThread.join();
    receiverThread.join();

    std::cout << "Program finished." << std::endl;

    return 0;
}