bin_PROGRAMS=host_receiver
host_receiver_SOURCES=dataprocessor.cpp host_receiver.cpp helper.cpp libusbinterface.cpp
host_receiver_SOURCES+=dataprocessor.h helper.h libusbinterface.h

host_receiver_CPPFLAGS = $(BOOST_CPPFLAGS)
host_receiver_LDADD = $(BOOST_REGEX_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_SYSTEM_LIBS)
host_receiver_LDFLAGS = $(BOOST_REGEX_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_SYSTEM_LDFLAGS) -lusb-1.0 -lreadline

host_receiver_CXXFLAGS= -std=c++11


if BUILD_PYMODULE
    lib_LTLIBRARIES=pyenergy.la
    pyenergy_la_SOURCES=$(host_receiver_SOURCES) pymodule.cpp
    pyenergy_la_CPPFLAGS = $(PYTHON_CPPFLAGS)
    pyenergy_la_LIBADD = $(BOOST_PYTHON_LIBS) $(BOOST_PYTHON3_LIBS) $(PYTHON_LIBS) $(host_receiver_LDADD)
    pyenergy_la_LDFLAGS = $(BOOST_PYTHON_LDFLAGS) $(BOOST_PYTHON3_LDFLAGS) -module -shared -avoid-version $(host_receiver_LDFLAGS)
endif

ACLOCAL_AMFLAGS=-Im4
