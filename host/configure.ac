#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([stm32-host-receiver], [0.0.1], [BUG-REPORT-ADDRESS])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE
AC_PROG_LIBTOOL
AC_CONFIG_SRCDIR([host_receiver.cpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Check for compiler flags.
AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

# Checks for libraries.
BOOST_REQUIRE([1.49.0])
BOOST_REGEX([n])
BOOST_THREADS([n])
BOOST_SYSTEM([n])

AC_CHECK_LIB([readline], [main],, [AC_MSG_ERROR([Cannot find readline library. Please install it.])])
AC_CHECK_LIB([usb-1.0], [main],, [AC_MSG_ERROR([Cannot find usb-1.0 library. Please install it.])])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_SSIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([sqrt])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
