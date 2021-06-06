include ../../common.mk

LIBNAME = libdwt
LIBPATH = $(ROOT)/src
CFLAGS += -I$(LIBPATH)
BIN = wavelet

.PHONY: all clean

all: $(BIN)

clean:
	$(MAKE) -C $(LIBPATH) $@
	-$(RM) $(BIN) *.o *.elf *.gdb

$(BIN): $(BIN).o $(LIBPATH)/$(LIBNAME).a

$(LIBPATH)/$(LIBNAME).a:
	$(MAKE) -C $(LIBPATH) $(LIBNAME).a

.PHONY: distclean
distclean: clean
	$(RM) *.mat *.dat

.PHONY: plot
plot:
	-gnuplot plot.txt
	-for f in *.svg; do test -s "$${f}" && inkscape -A "$${f/.svg/.pdf}" "$${f}"; done
