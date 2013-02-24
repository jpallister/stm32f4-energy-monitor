#include <stdio.h>
#include "libusbinterface.h"
#include "dataprocessor.h"
#include <boost/thread.hpp>
#include <queue>
#include <signal.h>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;

boost::thread thread1, thread2;
LibusbInterface *liObj;
DataProcessor *dpObj;

void ctrlc_handler(int _)
{
    liObj->endSignal();
    dpObj->endSignal();
}

int main()
{
    boost::mutex m;
    std::queue<boost::shared_array<unsigned char> > dQueue;
    LibusbInterface li(&m, &dQueue, 0x1337, 0x1337, "JP");
    DataProcessor dp(&m, &dQueue);

    liObj = &li;
    dpObj = &dp;

    signal(SIGINT, ctrlc_handler);

    thread1 = boost::thread(bind(&LibusbInterface::operator(),liObj));  // Bind prevents copying obj (need to keep ptr)
    thread2 = boost::thread(bind(&DataProcessor::operator(),dpObj));

    thread1.join();
    thread2.join();

    printf("Complete\n");

    return 0;
}
