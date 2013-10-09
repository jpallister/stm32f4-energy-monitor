#include <stdio.h>
#include <boost/thread.hpp>
#include "dataprocessor.h"
#include <time.h>
#include "libusbinterface.h"
#include "helper.h"
#include <math.h>

#define DEFAULT_OUTPUT "output_results"

using namespace std;
using namespace boost;
using namespace boost::accumulators;

DataProcessor::DataProcessor(boost::mutex *m, std::queue<DataSet> *d)
{
    mQueue = m;
    dQueue = d;
    status = RUNNING;
    cur_time = 0;
    doAccumulation = false;
    last_tick = 0;
    switched = false;
    isEmpty = true;
    resistor = 1.0;
    gain = 50.0;
    referenceVoltage = 3.0;
    /* Don't set output. This should be done with openOutput. */
    output = NULL;
}

DataProcessor::~DataProcessor()
{
    DataProcessor::closeOutput();
}

void DataProcessor::operator()()
{
    int t1, t2;

    t1 = time(0);
    while(status == RUNNING || !isEmpty)
    {
        getData();
        if(status != RUNNING && isEmpty)
            continue;
        processData();
        // printf("gda\n");

        t2 = time(0);
        if(switched && doAccumulation)
        {
            mt_start_output();
            printf("Avg: %4.1lf    Std:%3.1lf    Min: %4d    Max: %4d    kS/s:%3.1lf\n",mean(last_data),
                    sqrt(variance(last_data)), (int)extract::min(last_data), (int)extract::max(last_data),
                    extract::count(last_data)/1000.);
            mt_end_output();

            t1 = t2;
            switched = false;
        }
    }
}

void DataProcessor::getData()
{
    do {
        {
            boost::mutex::scoped_lock lock(*mQueue);

            if(!dQueue->empty())
            {
                isEmpty = false;
                data = dQueue->front();
                dQueue->pop();
                return;
            }
            isEmpty = true;
        }
        sleep(1);
    } while(status == RUNNING);

}

void DataProcessor::addDataItem(short val, unsigned long tstamp)
{
    if(tstamp > last_tick + TIMER_SECOND_TICKS)
    {
        last_data = current_data;
        // clear(current_data);
        current_data = decltype(current_data)();
        switched = true;
        last_tick += TIMER_SECOND_TICKS;
    }

    current_data(val);
}

/*
    This decodes the data that has been received. The data is 'compressed'. The
    first byte of each 64 byte transfer is the timer period between sequential
    samples. The remaining 63 bytes encodes 42 samples (2 samples / 3 bytes).

    0       Timer period
    3n+1    Low 8 bits of sample n*2
    3n+2    Low 8 bits of sample n*2+1
    3n+3    Low 4 bits  : bits 11-8 of sample n*2
            High 4 bits : bits 11-8 of sample n*2+1
 */
void DataProcessor::processData()
{
    int i, c = 0;
    short b1, b2;
    short rate = 0;
    float power;

    if (data.type == COMMAND)
    {
        switch (data.data[0])
        {
            case LibusbInterface::START:
                this->openOutput();
                break;
            case LibusbInterface::STOP:
                this->closeOutput();
                break;
        }
    }
    else
    {
        for(i = 0; i < DATA_LEN; ++i, ++c)
        {
            if(i % 64 == 0)
            {

                rate = data.data[i];
                // printf("%d\n", rate);
                c = 2;
            }
            else if(c % 3 == 0)
            {
                b1 = data.data[i];
            }
            else if(c % 3 == 1)
            {
                b2 = data.data[i];
            }
            else
            {
                b1 |= (data.data[i]&0x0F) << 8;
                b2 |= (data.data[i]&0xF0) << 4;

                power = convertToPower(b1);
                if (DataProcessor::openedFile())
                {
                    fprintf(output, "%f %lu\n", power, cur_time);
                }
                addDataItem(power, cur_time);
                cur_time += rate;

                power = convertToPower(b2);
                if (DataProcessor::openedFile())
                {
                    fprintf(output, "%f %lu\n", power, cur_time);
                }
                addDataItem(power, cur_time);
                cur_time += rate;
            }
        }
    }
}

void DataProcessor::endSignal()
{
    status = IDLE;
}


void DataProcessor::setAccumulation(bool sa)
{
    doAccumulation = sa;
}

void DataProcessor::setResistor(float res)
{
    resistor = res;
}

void DataProcessor::setReferenceVoltage(float v)
{
    referenceVoltage = v;
}

void DataProcessor::setGain(float v)
{
    gain = v;
}

float DataProcessor::convertToPower(float v)
{
    return double(v) * 4096. / 4095. / 4095. * double(referenceVoltage) * double(referenceVoltage)
        * 2. / double(gain) / double(resistor);
}

int DataProcessor::closeOutput()
{
    if (status == RUNNING)
    {
        return 1;
    }

    int to_return = 0;
    if (output)
    {
        fprintf(output,"\n");
        to_return = fclose(output);
        output = NULL;
    }

    return to_return;
}

int DataProcessor::openOutput()
{
    openOutput(DEFAULT_OUTPUT);
}

int DataProcessor::openOutput(std::string output_loc)
{
    int to_return = 0;
    if (!output)
    {
        output = fopen(output_loc.c_str(), "w");
        to_return = (output == NULL);
    }
    else
    {
        to_return = -1;
    }

    return to_return;
}

int DataProcessor::openedFile()
{
    return (output != NULL);
}
