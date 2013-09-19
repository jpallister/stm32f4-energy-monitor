#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([stm32f4-energy-monitor], [1.0], [james.pallister@embecosm.com])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE
AC_PROG_LIBTOOL
LT_INIT
AC_CONFIG_SRCDIR([src/host/host_receiver.cpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

AC_PROG_CXX
AC_PROG_CC

AC_PROG_CC([arm-none-eabi-gcc])

# TODO check if a suitable arm compiler exists
# Check to see if it can compile code for cortex-m4f
# Check to see if libopencm3 is available

# Check for compiler flags.
AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

# Checks for libraries.
BOOST_REQUIRE([1.49.0])
BOOST_REGEX([n])
BOOST_THREADS([n])
BOOST_SYSTEM([n])

# Save libraries (don't want to compile energy_monitor with them)
save_LIBS="$LIBS"
AC_CHECK_LIB([readline], [main], [], [AC_MSG_ERROR([Cannot find readline library. Please install it.])])
AC_CHECK_LIB([usb-1.0], [main], [], [AC_MSG_ERROR([Cannot find usb-1.0 library. Please install it.])])
LIBS="$save_LIBS"

# Python checking adapted from gdb's configure.ac
# It's license, and original code can be obtained from
# http://ftp.gnu.org/gnu/gdb/gdb-7.6.tar.bz2

AC_DEFUN([AC_TRY_LIBPYTHON],
  [
    version=$1
    define([have_libpython_var],$2)
    new_CPPFLAGS=$3
    new_LIBS=$4
    save_CPPFLAGS=$CPPFLAGS
    save_LIBS=$LIBS
    CPPFLAGS="$CPPFLAGS $new_CPPFLAGS"
    LIBS="$LIBS $new_LIBS"
    case $version in
        python3*)
          BOOST_FIND_LIB([python3], [n], [boost/python.hpp],
                         [], [BOOST_PYTHON_MODULE(empty) {}])
          ;;
        *)
          BOOST_PYTHON([n])
          ;;
    esac
    found_usable_python=no
    AC_MSG_CHECKING([for ${version}])
    AC_LINK_IFELSE([AC_LANG_SOURCE([AC_LANG_PROGRAM(
                    [[
                        #include "Python.h"
                    ]],

                    [[
                        Py_Initialize ();
                    ]])])],
                   [have_libpython_var=${version}
                    found_usable_python=yes
                    PYTHON_CPPFLAGS=$new_CPPFLAGS
                    PYTHON_LIBS=$new_LIBS])
    CPPFLAGS=$save_CPPFLAGS
    LIBS=$save_LIBS
    AC_MSG_RESULT([${found_usable_python}])
])

AC_ARG_WITH(python,
            AS_HELP_STRING([--with-python=PYTHON], [also build host receiver as a python module (/path/to/python)]),
            [],
            [with_python=`which python`])
AC_ARG_WITH(python,
            [AS_HELP_STRING([--without-python],[disable building of host receiver as a python module])],
            [],
            [with_python=no])

AC_MSG_CHECKING([whether to use python])
AC_MSG_RESULT([$with_python])

if test "${with_python}" = no; then
    AC_MSG_WARN([python module will not be compiled])
    have_libpython=no
else
    python_bin="${with_python}"
    if test ! -f "${python_bin}"; then
        AC_MSG_ERROR(unable to find python program ${python_bin})
    else
        python_includes=`${python_bin}-config --includes`
        if test $? != 0; then
            AC_MSG_ERROR(failure running python-config --includes)
        fi

        python_libs=`${python_bin}-config --ldflags`
        if test $? != 0; then
            AC_MSG_ERROR(failure running python-config --ldflags)
        fi

        have_libpython=no
        python_version=`echo " ${python_libs} " \
            | sed -e 's,^.* -l\(python[[0-9]]*[[\.]]*[[0-9]]*\).*$,\1,'`
        case "${python_version}" in
            python*)
                AC_TRY_LIBPYTHON(${python_version}, have_libpython,
                                 ${python_includes}, ${python_libs})
            ;;
            *)
                AC_MSG_ERROR([unable to determine python version from ${python_libs}])
            ;;
        esac

        if test "${have_libpython}" = no; then
            AC_MSG_ERROR([no usable python found at ${with_python}])
        fi
    fi
fi
AC_SUBST(PYTHON_CPPFLAGS)
AC_SUBST(PYTHON_LIBS)
AM_CONDITIONAL([BUILD_PYMODULE], [test x$have_libpython != xno])


AC_CONFIG_FILES([Makefile src/host/Makefile src/firmware/Makefile])
AC_OUTPUT
