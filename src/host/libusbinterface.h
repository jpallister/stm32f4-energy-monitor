#ifndef __LIBUSBINTERFACE_H__
#define __LIBUSBINTERFACE_H__
#include <queue>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>

#define DATA_LEN    2048

enum DataSetType {
    ENERGY_DATA = 0,
    COMMAND = 1
};

struct DataSet {
    boost::shared_array<unsigned char> data;
    DataSetType type;
};

class LibusbInterface
{
public:
    LibusbInterface(boost::mutex *, std::queue<DataSet> *, unsigned idVendor, unsigned idProduct, std::string serialId);
    ~LibusbInterface();
    void operator()();
    void endSignal();

    static std::vector<std::pair<std::string, std::string> > listDevices(unsigned idVendor, unsigned idProduct);

    enum CommandType {
        LED = 0,
        START = 1,
        STOP = 2,
        SETSERIAL = 3,
        SETTRIGGER = 4,
        SETMODE = 5,
        GETENERGY = 6
    };

    enum Mode {
        NORMAL_ADC = 0,
        DUAL_ADC = 1,
        OVERSAMPLED_ADC = 2
    };

    void sendCommand(CommandType);
    void setSerial(std::string);
    void setTrigger(char, int);
    void setMode(Mode);

    bool isRunning();

    /*  Energy in joules:
         =  Vref^2 / gain / resistor / 4096^2
               * tperiod * (2/168000000) * energy_accum
                             ^ peripheral clk rate (timer cnt)

        Elapsed time in seconds:
         = elapsed_time * (2/168000000)

        Peak power in watts
         = Vref^2 / gain / resistor / 4096^2 * peak_power

        Peak voltage in volts
          = peak_voltage / 4096 * Vref * 2

        Peak current in amps
          = peak_current / 4096 * Vref / gain / resistor
    */
    struct accumulated_data {
        uint64_t energy_accum;
        uint64_t elapsed_time;
        unsigned peak_power;
        unsigned peak_voltage;
        unsigned peak_current;
        unsigned n_samples;
    };
    accumulated_data lastData;

    bool cmdsEmpty();

private:
    boost::mutex *mQueue;
    boost::mutex cQueueMutex;
    std::queue<DataSet> *dQueue;

    // Attributes to look for in the USB devices
    unsigned idProduct, idVendor;
    std::string serialId;

    unsigned char data_buf[DATA_LEN];
    unsigned char interrupt_buf[64];
    int total_len;
    bool running;

    bool open_device();
    void close_device();

    // Send data to the data processor thread
    void send_data(DataSet);

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
