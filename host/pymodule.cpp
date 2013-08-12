#include <boost/python.hpp>
#include <libusb-1.0/libusb.h>
#include <readline/readline.h>

void processCommand(std::string input);

BOOST_PYTHON_MODULE(pyenergy)
{
    using namespace boost::python;

    int r;

    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        return;
    }

    rl_set_prompt("");

    def("processCommand", processCommand);
}
