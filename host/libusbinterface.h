#ifndef __LIBUSBINTERFACE_H__
#define __LIBUSBINTERFACE_H__
#include <queue>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>

#define DATA_LEN    2048

class LibusbInterface
{
public:
    LibusbInterface(boost::mutex *, std::queue<boost::shared_array<unsigned char> > *, unsigned idVendor, unsigned idProduct, std::string serialId);
    ~LibusbInterface();
    void operator()();
    void endSignal();

    static std::vector<std::pair<std::string, std::string> > listDevices(unsigned idVendor, unsigned idProduct);

    enum CommandType {
        LED = 0,
        START = 1,
        STOP = 2,
        SETSERIAL = 3,
        SETTRIGGER = 4
    };

    void sendCommand(CommandType);
    void setSerial(std::string);
    void setTrigger(char, int);

private:
    boost::mutex *mQueue;
    boost::mutex cQueueMutex;
    std::queue<boost::shared_array<unsigned char> > *dQueue;

    // Attributes to look for in the USB devices
    unsigned idProduct, idVendor;
    std::string serialId;

    unsigned char data_buf[DATA_LEN];
    unsigned char interrupt_buf[64];
    int total_len;
    bool running;

    bool open_device();
    void close_device();

    void send_start();
    void send_end();

    // Send data to the data processor thread
    void send_data(boost::shared_array<unsigned char>);

    int status;
    libusb_device_handle *devh;

    // Periodic bulk transfer of data from the device
    struct libusb_transfer *energy_transfer;
    static void LIBUSB_CALL transfer_callback(struct libusb_transfer *transfer);
    struct libusb_transfer *interrupt_transfer;
    static void LIBUSB_CALL interrupt_callback(struct libusb_transfer *transfer);

    // This class handles the sending on control information to the device

    struct CommandData {
        CommandType cmd;
        std::string cmd_data;
    };

    bool sendMonitorCommand(CommandData cmd);

    // Our command queue
    std::queue<CommandData> cQueue;
};

#endif
