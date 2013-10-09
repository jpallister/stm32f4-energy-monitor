#include <boost/python.hpp>
#include <libusb-1.0/libusb.h>
#include <readline/readline.h>
#include "host_receiver.h"
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>

using namespace boost::python;

int r;

class ReleaseGIL {
public:
    ReleaseGIL()
    {
        state = PyEval_SaveThread();
    }
    ~ReleaseGIL()
    {
        PyEval_RestoreThread(state);
    }

private:
    PyThreadState *state;

};

void connect_wrap()
{
    ReleaseGIL rg;

    cmd_connect();
}

bool is_running_wrap()
{
    ReleaseGIL rg;
    return cmd_is_running();
}

// TODO: Wrap other functions with ReleaseGIL (not sure if necessary)

BOOST_PYTHON_MODULE(pyenergy)
{
    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        return;
    }

//    PyEval_InitThreads();

    rl_set_prompt("");

    def("connect", connect_wrap);
    def("connect_to", cmd_connect_to);
    def("getserial", cmd_getserial);
    def("setserial", cmd_setserial);
    def("setresistor", cmd_setresistor);
    def("setrefvoltage", cmd_setrefvoltage);
    def("setgain", cmd_setgain);
    def("trigger", cmd_trigger);
    def("leds", cmd_leds);
    def("start", cmd_start);
    def("start_with_file", cmd_start_with_file);
    def("stop", cmd_stop);
    def("power", cmd_power);
    def("mode", cmd_mode);
    def("help", cmd_help);
    def("exit", cmd_exit);
    def("quit", cmd_exit);
    def("is_running",is_running_wrap);
}
