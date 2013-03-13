#ifndef __LIBUSBINTERFACE_H__
#define __LIBUSBINTERFACE_H__
#include <queue>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <string>
#include <libusb-1.0/libusb.h>

#define DATA_LEN    2048

class LibusbInterface
{
public:
    LibusbInterface(boost::mutex *, std::queue<boost::shared_array<unsigned char> > *, unsigned idVendor, unsigned idProduct, std::string serialId);
    ~LibusbInterface();
    void operator()();
    void endSignal();

    void sendCommand(int cmd);

private:
    boost::mutex *mQueue;
    std::queue<boost::shared_array<unsigned char> > *dQueue;
    unsigned idProduct, idVendor;
    std::string serialId;
    unsigned char data_buf[DATA_LEN];
    int total_len;

    bool open_device();
    void close_device();

    void send_start();
    void send_end();

    void send_data(boost::shared_array<unsigned char>);

    static void LIBUSB_CALL transfer_callback(struct libusb_transfer *transfer);

    int status;
    libusb_device_handle *devh;

    struct libusb_transfer *energy_transfer;

    class MonitorCommand
    {
    public:
        MonitorCommand(int cmd);

        void Send(libusb_device_handle *devh);
    private:
        int cmd_val;
    };

    std::queue<MonitorCommand> cQueue;
};

#endif
