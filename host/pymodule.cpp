#include <boost/python.hpp>
#include <libusb-1.0/libusb.h>
#include <readline/readline.h>
#include "host_receiver.h"

using namespace boost::python;

int r;

BOOST_PYTHON_MODULE(pyenergy)
{
    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        return;
    }

    rl_set_prompt("");

    def("connect", cmd_connect);
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
}
