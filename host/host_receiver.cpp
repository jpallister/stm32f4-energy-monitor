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
scoped_ptr<queue<shared_array<unsigned char> > > dQueue;
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
        auto devlist = LibusbInterface::listDevices(0xF539, 0xF539);
        int i = 0;

        if(devlist.empty())
        {
            cout << "    No devices found!" << endl;
            return;
        }

        if(devlist.size() > 1)
        {
            if(args.size() == 1)
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
                int devn = lexical_cast<int>(args[1]);
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
        }

        dQueue.reset(new queue<shared_array<unsigned char> >());
        liObj = new LibusbInterface(&bmutex, dQueue.get(), 0xF539, 0xF539, chosen_serial);
        dpObj = new DataProcessor(&bmutex, dQueue.get());

        thread1 = boost::thread(bind(&LibusbInterface::operator(),liObj));  // Bind prevents copying obj (need to keep ptr)
        thread2 = boost::thread(bind(&DataProcessor::operator(),dpObj));

        connected = true;
        cout << "    Connected to device, serial: " << chosen_serial << endl;
    }
    else if(args[0] == "getserial")
    {
        CHECK_CONNECTED();
        cout << "    Connected device, serial: " << chosen_serial << endl;
    }
    else if(args[0] == "setserial")
    {
        CHECK_CONNECTED();
        CHECK_PARAMS();

        if(args[1].length() != 8)
        {
            cout << "Length of the serial string must be 8 characters" << endl;
            return;
        }
        cout << "    Setting serial to " << args[1] << endl;
        liObj->setSerial(args[1]);
    }
    else if(args[0] == "trigger")
    {
        if(!connected)
        {
            cout << "    Need to be connected" << endl;
            return;
        }
        if(args.size() < 2)
        {
            cout << "Command expects parameters" << endl;
            return;
        }
        if(args[1] == "none")
        {
            cout << "    Removing trigger" << endl;
            liObj->setTrigger(0xFF, 0);
            return;
        }

        smatch sm;
        regex rx("P([A-H])(\\d{1,2})");
        if(regex_match(args[1], sm, rx))
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
            return;
        }
    }
    else if(args[0] == "leds")
    {
        CHECK_CONNECTED();
        liObj->sendCommand(LibusbInterface::LED);
    }
    else if(args[0] == "start")
    {
        CHECK_CONNECTED();
        liObj->sendCommand(LibusbInterface::START);
    }
    else if(args[0] == "stop")
    {
        CHECK_CONNECTED();
        liObj->sendCommand(LibusbInterface::STOP);
    }
    else if(args[0] == "power")
    {
        CHECK_CONNECTED();
        if(args.size() < 2)
        {
            dpObj->setAccumulation(true);
            return;
        }
        if(args[1] == "on")
            dpObj->setAccumulation(true);
        else
            dpObj->setAccumulation(false);
    }
    else if(args[0] == "mode")
    {
        CHECK_CONNECTED();
        CHECK_PARAMS();

        if(args[1] == "normal")
        {
            liObj->setMode(LibusbInterface::NORMAL_ADC);
        }
        else if(args[1] == "dual")
        {
            liObj->setMode(LibusbInterface::DUAL_ADC);
        }
        else if(args[1] == "oversampled")
        {
            liObj->setMode(LibusbInterface::OVERSAMPLED_ADC);
        }
    }
    else if(args[0] == "help")
    {
        cout << "Help" << endl;
        cout << "    connect                     Connect to the device (if only one attached)" << endl;
        cout << "             [SERIAL]           Select the device by serial number" << endl;
        cout << "    getserial                   Get the serial number of current device" << endl;
        cout << "    setserial SERIAL            Set the serial number of current device" << endl;
        cout << "                                Reconnection required before new serial recognised" << endl;
        cout << "    leds                        Toggle the LEDs" << endl;
        cout << "    start                       Start energy measurement" << endl;
        cout << "    stop                        Stop energy measurement" << endl;
        cout << "    trigger" << endl;
        cout << "             PIN                Trigger measurement on PIN (e.g PA0)" << endl;
        cout << "             none               Remove trigger" << endl;
        cout << "    mode " << endl;
        cout << "         normal                 Normal ADC mode" << endl;
        cout << "         dual                   Dual ADC, one voltage, one current, multiplied on board" << endl;
        cout << "         oversampled            Run ADC as fast as possible, average samples" << endl;
        cout << "    exit                        Exit the application" << endl;
        return;
    }
    else if(args[0] == "exit" || args[0] == "quit")
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
