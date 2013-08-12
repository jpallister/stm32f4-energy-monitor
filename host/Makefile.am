bin_PROGRAMS=host_receiver
lib_LTLIBRARIES=pyenergy.la
host_receiver_SOURCES=helper.cpp dataprocessor.cpp libusbinterface.cpp host_receiver.cpp
pyenergy_la_SOURCES=pymodule.cpp

host_receiver_CPPFLAGS = $(BOOST_CPPFLAGS)
host_receiver_LDADD = $(BOOST_REGEX_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_SYSTEM_LIBS)
host_receiver_LDFLAGS = $(BOOST_REGEX_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_SYSTEM_LDFLAGS)

pyenergy_la_CPPFLAGS = $(PYTHON_CPPFLAGS)
pyenergy_la_LIBADD = $(BOOST_PYTHON_LIBS) $(BOOST_PYTHON3_LIBS) $(PYTHON_LIBS)
pyenergy_la_LDFLAGS = $(BOOST_PYTHON_LDFLAGS) $(BOOST_PYTHON3_LDFLAGS) -module -shared -avoid-version

ACLOCAL_AMFLAGS=-Im4
