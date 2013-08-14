#ifndef __DATAPROCESSOR_H__
#define __DATAPROCESSOR_H__
#include <queue>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#define TIMER_SECOND_TICKS      84000000

namespace _ba =  boost::accumulators;

class DataProcessor
{
public:
    DataProcessor(boost::mutex *, std::queue<boost::shared_array<unsigned char> > *);
    ~DataProcessor();
    void operator()();
    void endSignal();

    void setAccumulation(bool);
    void setResistor(float);
    void setReferenceVoltage(float);
    void setGain(float);

    int closeOutput();
    int openOutput();
    int openOutput(std::string output_loc);
    int openedFile();
private:
    boost::mutex *mQueue;
    std::queue<boost::shared_array<unsigned char> > *dQueue;
    boost::shared_array<unsigned char> data;

    int status;
    bool isEmpty;

    void getData();
    void processData();

    FILE *output;
    unsigned long cur_time;

    bool doAccumulation;

    // Circuit parameters that are needed to calculate power
    float resistor;
    float referenceVoltage;
    float gain;

    float convertToPower(float v);

    void addDataItem(short, unsigned long);

    _ba::accumulator_set<double, _ba::features<_ba::tag::variance, _ba::tag::min, _ba::tag::max> > last_data, current_data;
    unsigned long last_tick;
    bool switched;
};

#endif
