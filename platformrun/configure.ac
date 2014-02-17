#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([platformrun], [1.0], [james.pallister@embecosm.com])

AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])

# ATMEGA328P Prerequisites ###################################################
AC_CHECK_PROG(AVROBJCOPY, avr-objcopy, avr-objcopy)
AC_CHECK_PROG(AVRDUDE, avrdude, avrdude)

# STM32F0DISCOVERY Prerequisites #############################################
AC_CHECK_PROG(ARMGDB, arm-none-eabi-gdb, arm-none-eabi-gdb)

AC_ARG_WITH([stlink], [AS_HELP_STRING([--with-stlink],
            [Specify directory of the st-util application])],
            [], [with_stlink=check])

if test x$with_stlink == xcheck; then
    AC_CHECK_PROG(STUTIL, st-util, st-util)
else
    AC_PATH_PROG(STUTIL, [st-util],[],[$with_stlink])
    if test x$STUTIL == x; then
        AC_MSG_FAILURE([--with-stlink was specified but st-util not found])
    fi
fi
#  TODO check for -c flag in stutil

# PIC32MX250F128B Prerequisites ##############################################
AC_CHECK_PROG(PIC32OBJCOPY, pic32-objcopy, pic32-objcopy)
AC_ARG_WITH([pic32prog], [AS_HELP_STRING([--with-pic32prog],
            [Specify directory of the pic32prog application])],
            [], [with_pic32prog=check])

if test x$with_pic32prog == xcheck; then
    AC_CHECK_PROG(PIC32PROG, pic32prog, pic32prog)
else
    AC_PATH_PROG(PIC32PROG, [pic32prog],[],[$with_pic32prog])
    if test x$PIC32PROG == x; then
        AC_MSG_FAILURE([--with-pic32prog was specified but picc32prog not found])
    fi
fi

# TI MSP Prerequisites #######################################################
AC_ARG_WITH([mspdebug], [AS_HELP_STRING([--with-mspdebug],
            [Specify directory of the mspdebug application])],
            [], [with_mspdebug=check])

if test x$with_mspdebug == xcheck; then
    AC_CHECK_PROG(MSPDEBUG, mspdebug, mspdebug)
else
    AC_PATH_PROG(MSPDEBUG, [mspdebug],[],[$with_mspdebug])
    if test x$MSPDEBUG == x; then
        AC_MSG_FAILURE([--with-mspdebug was specified but mspdebug not found])
    fi
fi


# Platform support, given programs we found above ############################

if test x$STUTIL != x && test x$ARMGDB != x; then
    stm32f0discovery_avail="yes"
    stm32vldiscovery_avail="yes"
else
    stm32f0discovery_avail="no"
    stm32vldiscovery_avail="no"
fi

if test x$AVROBJCOPY != x && test x$AVRDUDE != x; then
    atmega328p_avail="yes"
else
    atmega328p_avail="no"
fi

if test x$PIC32PROG != x && test x$PIC32OBJCOPY != x; then
    pic32mx250f128b_avail="yes"
else
    pic32mx250f128b_avail="no"
fi

if test x$MSPDEBUG != x; then
    mspexp430f5529_avail="yes"
    mspexp430fr5739_avail="yes"
else
    mspexp430f5529_avail="no"
    mspexp430fr5739_avail="no"
fi

# Check platform support against enables #####################################

AC_ARG_ENABLE([stm32f0discovery], [AS_HELP_STRING([--enable-stm32f0discovery], [Enable STM32F0DISCOVERY])],
        [stm32f0discovery=$enable_stm32f0discovery], [stm32f0discovery=check])
AC_ARG_ENABLE([stm32vldiscovery], [AS_HELP_STRING([--enable-stm32vldiscovery], [Enable STM32VLDISCOVERY])],
        [stm32vldiscovery=$enable_stm32vldiscovery], [stm32vldiscovery=check])
AC_ARG_ENABLE([atmega328p], [AS_HELP_STRING([--enable-atmega328p], [Enable ATMEGA328P])],
        [atmega328p=$enable_atmega328p], [atmega328p=check])
AC_ARG_ENABLE([pic32mx250f128b], [AS_HELP_STRING([--enable-pic32mx250f128b], [Enable PIC32MX250F128B])],
        [pic32mx250f128b=$enable_pic32mx250f128b], [pic32mx250f128b=check])
AC_ARG_ENABLE([mspexp430f5529], [AS_HELP_STRING([--enable-mspexp430f5529], [Enable MSP-EXP430F5529])],
        [mspexp430f5529=$enable_mspexp430f5529], [mspexp430f5529=check])
AC_ARG_ENABLE([mspexp430fr5739], [AS_HELP_STRING([--enable-mspexp430fr5739], [Enable MSP-EXP430FR5739])],
        [mspexp430fr5739=$enable_mspexp430fr5739], [mspexp430fr5739=check])

# If we explicitly enable or disable it ######################################

if test x$stm32f0discovery == xcheck; then
    stm32f0discovery=$stm32f0discovery_avail
elif test x$stm32f0discovery_avail == xno && test x$stm32f0discovery == xyes; then
    AC_MSG_FAILURE([The stm32f0discovery platform is enabled but its prerequisites were not satisfied])
fi

if test x$stm32vldiscovery == xcheck; then
    stm32vldiscovery=$stm32vldiscovery_avail
elif test x$stm32vldiscovery_avail == xno && test x$stm32vldiscovery ==xyes; then
    AC_MSG_FAILURE([The stm32vldiscovery platform is enabled but its prerequisites were not satisfied])
fi

if test x$atmega328p == xcheck; then
    atmega328p=$atmega328p_avail
elif test x$atmega328p_avail == xno && test x$atmega328p == xyes; then
    AC_MSG_FAILURE([The atmega328p platform is enabled but its prerequisites were not satisfied])
fi

if test x$pic32mx250f128b == xcheck; then
    pic32mx250f128b=$pic32mx250f128b_avail
elif test x$pic32mx350f128b_avail == xno && test x$pic32mx250f128b == xyes; then
    AC_MSG_FAILURE([The pic32mx250f128b platform is enabled but its prerequisites were not satisfied])
fi

if test x$mspexp430f5529 == xcheck; then
    mspexp430f5529=$mspexp430f5529_avail
elif test x$mspexp430f5529_avail == xno && test x$mspexp430f5529 == xyes; then
    AC_MSG_FAILURE([The msp-exp430f5529 platform is enabled but its prerequisites were not satisfied])
fi

if test x$mspexp430fr5739 == xcheck; then
    mspexp430fr5739=$mspexp430fr5739_avail
elif test x$mspexp430fr5739_avail == xno && test x$mspexp430fr5739 == xyes; then
    AC_MSG_FAILURE([The msp-exp430fr5739 platform is enabled but its prerequisites were not satisfied])
fi

# Pass variables to makefile #################################################

AS_IF([test x$stm32f0discovery == xyes], [AC_SUBST(STM32F0DISCOVERY, "True")], [AC_SUBST(STM32F0DISCOVERY, "False")])
AS_IF([test x$stm32vldiscovery == xyes], [AC_SUBST(STM32VLDISCOVERY, "True")], [AC_SUBST(STM32VLDISCOVERY, "False")])
AS_IF([test x$atmega328p == xyes], [AC_SUBST(ATMEGA328P, "True")], [AC_SUBST(ATMEGA328P, "False")])
AS_IF([test x$pic32mx250f128b == xyes], [AC_SUBST(PIC32MX250F128B, "True")], [AC_SUBST(PIC32MX250F128B, "False")])
AS_IF([test x$mspexp430f5529 == xyes], [AC_SUBST(MSPEXP430F5529, "True")], [AC_SUBST(MSPEXP430F5529, "False")])
AS_IF([test x$mspexp430fr5739 == xyes], [AC_SUBST(MSPEXP430FR5739, "True")], [AC_SUBST(MSPEXP430FR5739, "False")])

AC_CONFIG_FILES([platformrun.py])
AC_OUTPUT

# List configured options ####################################################

echo ""
echo ""
echo "*** Platformrun ***"
echo ""
echo "stm32f0discovery                              $stm32f0discovery"
echo "stm32vldiscovery                              $stm32vldiscovery"
echo "atmega328p                                    $atmega328p      "
echo "pic32mx250f128b                               $pic32mx250f128b "
echo "msp-exp430f5529                               $mspexp430f5529  "
echo "msp-exp430fr5739                              $mspexp430fr5739 "


