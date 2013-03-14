#include <stdio.h>
#include <boost/thread.hpp>
#include "dataprocessor.h"
#include <time.h>
#include "libusbinterface.h"

using namespace boost;

DataProcessor::DataProcessor(boost::mutex *m, std::queue<boost::shared_array<unsigned char> > *d)
{
    mQueue = m;
    dQueue = d;
    status = 0;
    output = fopen("output_results", "w");
    cur_time = 0;
}

DataProcessor::~DataProcessor()
{
    if(output)
        fclose(output);
}

void DataProcessor::operator()()
{
    int total = 0;
    int t1, t2;

    t1 = time(0);
    while(status == 0)
    {
        getData();
        if(status != 0)
            continue;
        processData();
        total += DATA_LEN;
        // printf("gda\n");

        t2 = time(0);
        if(t2 - t1 >= 2)
        {
            // printf("D %f\n",(float)total/(t2-t1));
            t1 = t2;
            total = 0;
        }
    }
}

void DataProcessor::getData()
{
    do {
        {
            boost::mutex::scoped_lock(*mQueue);

            if(!dQueue->empty())
            {
                data = dQueue->front();
                dQueue->pop();
                return;
            }
        }
        sleep(1);
    } while(status == 0);

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

    for(i = 0; i < DATA_LEN; ++i, ++c)
    {
        if(i % 64 == 0)
        {

            rate = data[i];
            // printf("%d\n", rate);
            c = 2;
        }
        else if(c % 3 == 0)
        {
            b1 = data[i];
        }
        else if(c % 3 == 1)
        {
            b2 = data[i];
        }
        else
        {
            b1 |= (data[i]&0x0F) << 8;
            b2 |= (data[i]&0xF0) << 4;
            fprintf(output, "%d %lu\n", b1, cur_time);
            cur_time += rate;
            fprintf(output, "%d %lu\n", b2, cur_time);
            cur_time += rate;
        }
    }
}

void DataProcessor::endSignal()
{
    status = 1;
}
