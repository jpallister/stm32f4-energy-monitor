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

using namespace std;
using namespace boost;

boost::thread thread1, thread2;
LibusbInterface *liObj;
DataProcessor *dpObj;
bool running = true;

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
        if(args.size() < 2)
        {
            cout << "Command expects parameters" << endl;
            return;
        }
        liObj->sendCommand(lexical_cast<int>(args[1]));
    }
    else
    {
        cout << " *** Unrecognised command \"" << args[0] << "\"" << endl;
    }
}

int main()
{
    boost::mutex m;
    std::queue<boost::shared_array<unsigned char> > dQueue;
    LibusbInterface li(&m, &dQueue, 0x1337, 0x1337, "JP");
    DataProcessor dp(&m, &dQueue);
    string input;

    liObj = &li;
    dpObj = &dp;

    signal(SIGINT, ctrlc_handler);

    thread1 = boost::thread(bind(&LibusbInterface::operator(),liObj));  // Bind prevents copying obj (need to keep ptr)
    thread2 = boost::thread(bind(&DataProcessor::operator(),dpObj));

    while(running)
    {
        cout << "> ";
        getline(cin, input);

        if(input == "exit" || input == "\x0D")
        {
            cout << "Exiting...\n";
            liObj->endSignal();
            dpObj->endSignal();
            running = false;
        }
        else
            processCommand(input);
    }

    thread1.join();
    thread2.join();

    printf("Complete\n");

    return 0;
}
