#ifndef __DATAPROCESSOR_H__
#define __DATAPROCESSOR_H__
#include <queue>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>

class DataProcessor
{
public:
    DataProcessor(boost::mutex *, std::queue<boost::shared_array<unsigned char> > *);
    ~DataProcessor();
    void operator()();
    void endSignal();

    void setAccumulation(bool);
private:
    boost::mutex *mQueue;
    std::queue<boost::shared_array<unsigned char> > *dQueue;
    boost::shared_array<unsigned char> data;

    int status;

    void getData();
    void processData();

    FILE *output;
    unsigned long cur_time;

    bool doAccumulation;
    unsigned long long total;
    unsigned count;
};

#endif
