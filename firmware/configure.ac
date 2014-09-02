#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([stm32f4-energy-monitor], [1.0], [james.pallister@embecosm.com])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])
AC_CANONICAL_SYSTEM
AC_CANONICAL_TARGET
AM_INIT_AUTOMAKE


AM_MAINTAINER_MODE([disable])

if test x$host_alias != xarm-none-eabi; then
    AC_MSG_ERROR([Please rerun with --host=arm-none-eabi])
fi

libopencm3_location=`readlink -m $srcdir`/libopencm3

if ! test -d $libopencm3_location; then
    echo -e "Please run (in the source directory)\n\tcd firmware\n\tgit clone https://github.com/libopencm3/libopencm3\n\tcd libopencm3\n\tgit checkout 806ebb18faf3285bb4dfa3c9c2caeac77dca7f34\n\tmake"
    exit 1;
fi

# gcc-arm-embedded requires these libraries for the tests, else they'll fail
AC_SUBST([LDFLAGS], ["-lc -lnosys -L$libopencm3_location/lib $LDFLAGS"])
AC_SUBST([CFLAGS], ["-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I$libopencm3_location/include $CFLAGS"])

AC_PROG_LIBTOOL
LT_INIT
AC_CONFIG_SRCDIR([src/])

AC_PROG_CC

#AC_PROG_CC([arm-none-eabi-gcc])

AC_CHECK_LIB(opencm3_stm32f4, nvic_enable_irq, [], [AC_MSG_ERROR([libopencm3_stm32f4 not found])])

AC_PATH_PROG(STUTIL, [st-util])
AC_PATH_TOOL(ARMGDB, gdb)

echo $STUTIL
echo $ARMGDB

if test x$ARMGDB == xgdb; then
    AS_UNSET(ARMGDB)
fi

AM_CONDITIONAL([AUTOFLASH], [test x$STUTIL != x && test x$ARMGDB != x])

AC_CONFIG_FILES([Makefile src/Makefile])
AC_OUTPUT
