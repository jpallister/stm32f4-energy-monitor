#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include "libusbinterface.h"
#include <string>
#include <boost/bind.hpp>
#include <time.h>

using namespace boost;
// using namespace boost::bind;

LibusbInterface::LibusbInterface(boost::mutex *m, std::queue<boost::shared_array<unsigned char> > *d,
    unsigned idVendor, unsigned idProduct, std::string serialId)
{
    mQueue = m;
    dQueue = d;
    this->idVendor = idVendor;
    this->idProduct = idProduct;
    this->serialId = serialId;
    status = 0;
    total_len = 0;
}

LibusbInterface::~LibusbInterface()
{
    printf("Destructr\n");
}

void LibusbInterface::operator()()
{
    int r;
    int t1, t2;

    if(open_device())
        printf("Opened\n");

    energy_transfer = libusb_alloc_transfer(0);
    if(!energy_transfer)
    {
        printf("Couldn't allocate a transfer\n");
        return;
    }

    libusb_fill_bulk_transfer(energy_transfer, devh, (1 | LIBUSB_ENDPOINT_IN), data_buf, sizeof(data_buf), &LibusbInterface::transfer_callback, this, 500);

    if(status == 0)
        send_start();

    if((r=libusb_submit_transfer(energy_transfer)) < 0)
    {
        printf("Submit transfer error: %d\n", r);
        return;
    }

    t1 = time(0);
    while(status == 0)
    {
        r = libusb_handle_events(NULL);

        if(r < 0)
        {
            printf("Error?\n");
            status = -1;
        }

        t2 =  time(0);

        if(t2 - t1 >= 2)
        {
            // printf("%f\n", (float)total_len/(t2-t1));
            t1=t2;
            total_len = 0;
        }

        if(!cQueue.empty())
        {
            cQueue.front().Send(devh);
            cQueue.pop();
        }
    }

    if(devh)
        send_end();

    close_device();
}

bool LibusbInterface::open_device()
{
    libusb_device **devs, *dev;
    int i = 0, r;
    ssize_t cnt;

    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        status = -1;
        return false;
    }

    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0)
    {
        printf("Could not get device list: %d\n", (int)cnt);
        status = -1;
        return false;
    }

    while ((dev = devs[i++]) != NULL)
    {
        struct libusb_device_descriptor desc;

        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0)
        {
            printf("Failed to get device descriptor: %d\n", r);
            status = -1;
            return false;
        }

        if(desc.idVendor == idVendor && desc.idProduct == idProduct)
        {
            unsigned char sId[256];

            r = libusb_open(dev, &devh);
            if(r < 0)
            {
                printf("Failed to open device: %d\n", r);
                status = -1;
                return false;
            }

            r = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, sId, 256);
            if(r < 0)
            {
                printf("Failed to get string descriptor: %d\n", r);
                status = -1;
                return false;
            }

            if(serialId == std::string((char*)sId))
            {
                r = libusb_claim_interface(devh, 0);
                if(r < 0)
                {
                    printf("Error claiming interface %d\n", r);
                    status = -1;
                    return false;
                }

                return true;
            }
        }
    }

    printf("Could not find device...\n");
    status = -1;
    return false;
}

void LibusbInterface::send_start()
{
    unsigned char buf[] = "S";
    int len;

    libusb_bulk_transfer(devh, 0x1, buf, 1, &len, 0);
}

void LibusbInterface::send_end()
{
    unsigned char buf[] = "F";
    int len;

    libusb_bulk_transfer(devh, 0x1, buf, 1, &len, 0);
}

void LibusbInterface::close_device()
{
    if(devh)
    {
        int len, r;
        unsigned char buf[2];

        do
        {
            r = libusb_bulk_transfer(devh, 0x81, buf, 1, &len, 100);
        } while(len > 0 && r >= 0);

        libusb_release_interface(devh, 0);
        libusb_close(devh);
    }
}

void LibusbInterface::send_data(shared_array<unsigned char> data)
{
    boost::mutex::scoped_lock(*mQueue);

    dQueue->push(data);
}


void LIBUSB_CALL LibusbInterface::transfer_callback(struct libusb_transfer *transfer)
{
    LibusbInterface * _this = (LibusbInterface*)transfer->user_data;
    int r;

    if(transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        printf("Transfer status: %d\n", transfer->status);
        libusb_free_transfer(transfer);
        _this->energy_transfer = NULL;
        return;
    }

    // printf("cb\n");
    _this->total_len += DATA_LEN;

    shared_array<unsigned char> data(new unsigned char[DATA_LEN]);

    memcpy(data.get(), _this->data_buf, DATA_LEN);
    _this->send_data(data);

    if((r=libusb_submit_transfer(transfer)) < 0)
    {
        printf("Submit transfer error: %d\n", r);
        return;
    }
}

void LibusbInterface::endSignal()
{
    status = 1;
}

void LibusbInterface::sendCommand(int cmd)
{
    // Queue commands, because this can be from any thread
    cQueue.push(MonitorCommand(cmd));
}



LibusbInterface::MonitorCommand::MonitorCommand(int cmd)
{
    cmd_val = cmd;
}

void LibusbInterface::MonitorCommand::Send(libusb_device_handle *devh)
{
    // Use synchronous IO for this (should be short?)
    libusb_control_transfer(devh, 65, cmd_val, 0, 0, NULL, 0, 0);
}