bin_PROGRAMS=host_receiver
host_receiver_SOURCES=helper.cpp dataprocessor.cpp libusbinterface.cpp host_receiver.cpp

host_receiver_CPPFLAGS = $(BOOST_CPPFLAGS)
host_receiver_LDADD = $(BOOST_REGEX_LIBS) $(BOOST_THREAD_LIBS) $(BOOST_SYSTEM_LIBS)
host_receiver_LDFLAGS = $(BOOST_REGEX_LDFLAGS) $(BOOST_THREAD_LDFLAGS) $(BOOST_SYSTEM_LDFLAGS)

ACLOCAL_AMFLAGS=-Im4
