#include <stdio.h>
#include "libusbinterface.h"
#include "dataprocessor.h"
#include <boost/thread.hpp>
#include <queue>
#include <vector>
#include <string>
#include <signal.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>

using namespace std;
using namespace boost;

boost::mutex bmutex;
scoped_ptr<queue<shared_array<unsigned char> > > dQueue;
scoped_ptr<LibusbInterface> liObj;
scoped_ptr<DataProcessor> dpObj;
boost::thread thread1, thread2;
bool running = true;

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
        // if(args.size() < 2)
        // {
        //     cout << "Command expects parameters" << endl;
        //     return;
        // }
        // liObj->sendCommand(lexical_cast<int>(args[1]));
    }
    else if (args[0] == "connect")
    {
        auto devlist = LibusbInterface::listDevices(0x1337, 0x1337);
        string serial;
        int i = 0;

        if(devlist.empty())
            cout << "    No devices found!" << endl;

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
                serial = devlist[devn].first;
            }
        }
        else
        {
            serial = devlist[0].first;
        }

        if(connected)
        {
            liObj->endSignal();
            dpObj->endSignal();
            connected = false;
            thread1.join();
            thread2.join();
        }

        dQueue.reset(new queue<shared_array<unsigned char> >());
        liObj.reset(new LibusbInterface(&bmutex, dQueue.get(), 0x1337, 0x1337, serial));
        dpObj.reset(new DataProcessor(&bmutex, dQueue.get()));

        thread1 = boost::thread(bind(&LibusbInterface::operator(),liObj.get()));  // Bind prevents copying obj (need to keep ptr)
        thread2 = boost::thread(bind(&DataProcessor::operator(),dpObj.get()));

        connected = true;
    }
    else if(args[0] == "getserial")
    {

    }
    else if(args[0] == "setserial")
    {

    }
    else if(args[0] == "trigger")
    {
        if(args.size() < 2)
        {
            cout << "Command expects parameters" << endl;
            return;
        }
    }
    else if(args[0] == "leds")
    {
        liObj->sendCommand(LibusbInterface::LED);
    }
    else if(args[0] == "start")
    {
        liObj->sendCommand(LibusbInterface::START);
    }
    else if(args[0] == "stop")
    {
        liObj->sendCommand(LibusbInterface::STOP);
    }
    else if(args[0] == "help")
    {
        cout << "Help" << endl;
        cout << "    connect                     Connect to the device (if only one attached)" << endl;
        cout << "             [SERIAL]           Select the device by serial number" << endl;
        cout << "    getserial                   Get the serial number of current device" << endl;
        cout << "    setserial SERIAL            Set the serial number of current device" << endl;
        cout << "    trigger " << endl;
        cout << "             (start | stop)     Start/stop the energy measurement" << endl;
        cout << "             pin PIN            The energy measurement is started/stopped on PIN" << endl;
        cout << "    mode " << endl;
        cout << "             normal" << endl;
        cout << "             dual" << endl;
        cout << "             oversampled" << endl;
        cout << "    exit                       Exit the application" << endl;
        return;
    }
    else
    {
        cout << " *** Unrecognised command \"" << args[0] << "\"" << endl;
    }
}

int main()
{
    string input;
    int r;

    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        return false;
    }

    while(running)
    {
        cout << "> ";
        getline(cin, input);

        if(input == "exit" || input == "\x0D" || input == "quit")
        {
            cout << "Exiting...\n";
            if(connected)
            {
                liObj->endSignal();
                dpObj->endSignal();
                running = false;
                connected = false;
            }
        }
        else
            processCommand(input);
    }

    thread1.join();
    thread2.join();

    printf("Complete\n");

    return 0;
}
