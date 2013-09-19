#include <stdio.h>
#include <boost/thread.hpp>
#include <queue>
#include <vector>
#include <string>
#include <boost/regex.hpp>
#include <signal.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <readline/readline.h>
#include <readline/history.h>
#include <sys/types.h>

#include "config.h"
#include "dataprocessor.h"
#include "libusbinterface.h"
#include "host_receiver.h"

#define CHECK_CONNECTED() \
    if (!connected) { \
        cout << "    Need to be connected" << endl; \
        return; \
    }

#define CHECK_PARAMS() \
    if (args.size() < 2) { \
        cout << "Command expects parameters" << endl; \
        return; \
    }

using namespace std;
using namespace boost;

boost::mutex bmutex;
scoped_ptr<queue<DataSet> > dQueue;
LibusbInterface *liObj;
DataProcessor *dpObj;
boost::thread thread1, thread2;
bool running = true;
string chosen_serial;

bool connected = false;

void ctrlc_handler(int _)
{
    liObj->endSignal();
    dpObj->endSignal();
    running = false;
}

void cmd_connect()
{
    cmd_connect_to("");
}

void cmd_connect_to(string connect_to)
{
    auto devlist = LibusbInterface::listDevices(0xF539, 0xF539);
    int i = 0;

    if(devlist.empty())
    {
        cout << "    No devices found!" << endl;
        return;
    }

    if(devlist.size() > 1)
    {
        if(connect_to.compare(""))
        {
            cout << "    Select a device" << endl;
            for(auto dev : devlist)
            {
                i++;
                cout << "        " << i << ": " << dev.first << "\t" << dev.second << endl;
            }
            return;
        }
        else
        {
            int devn = lexical_cast<int>(connect_to);
            chosen_serial = devlist[devn].first;
        }
    }
    else
    {
        chosen_serial = devlist[0].first;
    }

    if(connected)
    {
        liObj->endSignal();
        dpObj->endSignal();
        connected = false;
        thread1.join();
        thread2.join();
        delete liObj;
        delete dpObj;
        dpObj = NULL;
    }

    dQueue.reset(new queue<DataSet>());
    liObj = new LibusbInterface(&bmutex, dQueue.get(), 0xF539, 0xF539, chosen_serial);
    dpObj = new DataProcessor(&bmutex, dQueue.get());

    thread1 = boost::thread(bind(&LibusbInterface::operator(),liObj));  // Bind prevents copying obj (need to keep ptr)
    thread2 = boost::thread(bind(&DataProcessor::operator(),dpObj));

    connected = true;
    cout << "    Connected to device, serial: " << chosen_serial << endl;
}

void cmd_getserial()
{
    CHECK_CONNECTED();
    cout << "    Connected device, serial: " << chosen_serial << endl;
}

void cmd_setserial(string new_serial)
{
    CHECK_CONNECTED();

    if(new_serial.length() != 8)
    {
        cout << "Length of the serial string must be 8 characters" << endl;
        return;
    }
    cout << "    Setting serial to " << new_serial << endl;
    liObj->setSerial(new_serial);
}

void cmd_setresistor(string new_resist)
{
    smatch sm;
    regex rx("([0-9.]+)");

    if(regex_match(new_resist, sm, rx))
    {
        float res_val = lexical_cast<float>(sm[1]);

        cout << "    Setting resistor value to " << res_val << "Ω" << endl;
        dpObj->setResistor(res_val);
    }
    else
    {
        cout << "Invalid resistor value" << endl;
    }
}

void cmd_setrefvoltage(string new_voltage)
{
    smatch sm;
    regex rx("([0-9.]+)");

    if(regex_match(new_voltage, sm, rx))
    {
        float res_val = lexical_cast<float>(sm[1]);

        cout << "    Setting reference voltage to " << res_val << "V" << endl;
        dpObj->setReferenceVoltage(res_val);
    }
    else
    {
        cout << "Invalid reference voltage" << endl;
    }
}

void cmd_setgain(string new_gain)
{
    smatch sm;
    regex rx("([0-9.]+)");

    if(regex_match(new_gain, sm, rx))
    {
        float res_val = lexical_cast<float>(sm[1]);

        cout << "    Setting gain to " << res_val << endl;
        dpObj->setGain(res_val);
    }
    else
    {
        cout << "Invalid gain value" << endl;
        return;
    }
}

void cmd_trigger(string trigger)
{
    CHECK_CONNECTED();
    if(trigger == "none")
    {
        cout << "    Removing trigger" << endl;
        liObj->setTrigger(0xFF, 0);
        return;
    }

    smatch sm;
    regex rx("P([A-H])(\\d{1,2})");
    if(regex_match(trigger, sm, rx))
    {
        int pinnum = lexical_cast<int>(sm[2]);

        if(pinnum > 16)
            cout << "Pin number should be 0-15";
        else
        {
            cout << "    Setting trigger to port P" << sm[1] << sm[2] << endl;
            liObj->setTrigger(sm[1].str()[0], pinnum);
        }
    }
    else
    {
        cout << "Invalid portname" << endl;
    }
}

void cmd_leds()
{
    CHECK_CONNECTED();
    liObj->sendCommand(LibusbInterface::LED);
}

void cmd_start()
{
    CHECK_CONNECTED();
    liObj->sendCommand(LibusbInterface::START);
}

void cmd_start_with_file(string output_file)
{
    CHECK_CONNECTED();

    dpObj->openOutput(output_file);
    liObj->sendCommand(LibusbInterface::START);
}

void cmd_stop()
{
    CHECK_CONNECTED();

    liObj->sendCommand(LibusbInterface::STOP);
}

void cmd_power()
{
    CHECK_CONNECTED();
    dpObj->setAccumulation(true);
}

void cmd_power_set(string power)
{
    CHECK_CONNECTED();

    if(power == "on")
        dpObj->setAccumulation(true);
    else
        dpObj->setAccumulation(false);
}

void cmd_mode(string new_mode)
{
    CHECK_CONNECTED();

    if(new_mode == "normal")
    {
        liObj->setMode(LibusbInterface::NORMAL_ADC);
    }
    else if(new_mode == "dual")
    {
        liObj->setMode(LibusbInterface::DUAL_ADC);
    }
    else if(new_mode == "oversampled")
    {
        liObj->setMode(LibusbInterface::OVERSAMPLED_ADC);
    }
    else
    {
        cout << "Invalid mode!" << endl;
    }
}

void cmd_help()
{
    cout << "Help" << endl;
    cout << "    connect                     Connect to the device (if only one attached)" << endl;
    cout << "             [SERIAL]           Select the device by serial number" << endl;
    cout << "    getserial                   Get the serial number of current device" << endl;
    cout << "    setserial SERIAL            Set the serial number of current device" << endl;
    cout << "                                Reconnection required before new serial recognised" << endl;
    cout << "    leds                        Toggle the LEDs" << endl;
    cout << "    setresistor RESISTOR        Set the value of the shunt resistor" << endl;
    cout << "    setrefvoltage REFVOLTAGE    Set the value of the ADC reference voltage" << endl;
    cout << "    setgain GAIN                Set the gain of the high side current amplifier" << endl;
    cout << "    start [FILE]                Start energy measurement, collecting results into FILE (output_results by default)" << endl;
    cout << "    stop                        Stop energy measurement" << endl;
    cout << "    trigger" << endl;
    cout << "             PIN                Trigger measurement on PIN (e.g PA0)" << endl;
    cout << "             none               Remove trigger" << endl;
    cout << "    mode " << endl;
    cout << "         normal                 Normal ADC mode" << endl;
    cout << "         dual                   Dual ADC, one voltage, one current, multiplied on board" << endl;
    cout << "         oversampled            Run ADC as fast as possible, average samples" << endl;
    cout << "    exit                        Exit the application" << endl;
}

void cmd_exit()
{
    cout << "Exiting...\n";
    if(connected)
    {
        liObj->endSignal();
        dpObj->endSignal();
        connected = false;
        thread1.join();
        thread2.join();
        delete liObj;
        delete dpObj;
    }
    running = false;
}

void processCommand(string input)
{
    vector<string> args;

    split(args, input, is_any_of(" \t"));

    if(args[0] == "cmd")
    {
        // CHECK_PARAMS();
        // liObj->sendCommand(lexical_cast<int>(args[1]));
    }
    else if (args[0] == "connect")
    {
        if (args.size() == 1)
        {
            cmd_connect();
        }
        else
        {
            cmd_connect_to(args[1]);
        }
    }
    else if(args[0] == "getserial")
    {
        cmd_getserial();
    }
    else if(args[0] == "setserial")
    {
        CHECK_PARAMS();
        cmd_setserial(args[1]);
    }
    else if(args[0] == "setresistor")
    {
        CHECK_PARAMS();
        cmd_setresistor(args[1]);
    }
    else if(args[0] == "setrefvoltage")
    {
        CHECK_PARAMS();
        cmd_setrefvoltage(args[1]);
    }
    else if(args[0] == "setgain")
    {
        CHECK_PARAMS();
        cmd_setgain(args[1]);
    }
    else if(args[0] == "trigger")
    {
        CHECK_PARAMS();
        cmd_trigger(args[1]);
    }
    else if(args[0] == "leds")
    {
        cmd_leds();
    }
    else if(args[0] == "start")
    {
        if (args.size() == 1)
        {
            cmd_start();
        }
        else
        {
            cmd_start_with_file(args[1]);
        }
    }
    else if(args[0] == "stop")
    {
        cmd_stop();
    }
    else if(args[0] == "power")
    {
        if (args.size() == 1)
        {
            cmd_power();
        }
        else
        {
            cmd_power_set(args[1]);
        }
    }
    else if(args[0] == "mode")
    {
        CHECK_PARAMS();
        cmd_mode(args[1]);
    }
    else if(args[0] == "help")
    {
        cmd_help();
    }
    else if(args[0] == "exit" || args[0] == "quit")
    {
        cmd_exit();
    }
    else
    {
        cout << " *** Unrecognised command \"" << args[0] << "\"" << endl;
    }
}

int main()
{
    char *input;
    int r;

    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        return false;
    }

    rl_bind_key('\t',rl_abort);

    while(running)
    {
        input = readline("> ");

        if(input == NULL)
            continue;

        processCommand(input);
        add_history(input);
        free(input);
    }

    printf("Complete\n");

    return 0;
}
