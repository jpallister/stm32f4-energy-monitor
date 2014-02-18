#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([stm32f4-energy-monitor], [1.0], [james.pallister@embecosm.com])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE

AM_MAINTAINER_MODE([disable])

if test x$host_alias != xarm-none-eabi; then
    AC_MSG_ERROR([Please rerun with --host=arm-none-eabi])
fi

# gcc-arm-embedded requires these libraries for the tests, else they'll fail
AC_SUBST([LDFLAGS], ["-lc -lnosys $LDFLAGS"])
AC_SUBST([CFLAGS], ["-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $CFLAGS"])

AC_PROG_LIBTOOL
LT_INIT
AC_CONFIG_SRCDIR([firmware/])

AC_PROG_CC

#AC_PROG_CC([arm-none-eabi-gcc])

AC_CHECK_LIB(opencm3_stm32f4, nvic_enable_irq, [], [AC_MSG_ERROR([libopencm3_stm32f4 not found])])

AC_CONFIG_FILES([Makefile firmware/Makefile])
AC_OUTPUT
