SUBDIRS=src

ACLOCAL_AMFLAGS= -I m4

if AUTOFLASH
autoflash: all
	@echo Starting GDB server
	$(STUTIL) -p 42424 -c 0x2ba01477 -v0 > /dev/null&
	@echo Starting GDB
	$(ARMGDB) src/energy_monitor -batch -ex "tar ext :42424" -ex "load"
endif
