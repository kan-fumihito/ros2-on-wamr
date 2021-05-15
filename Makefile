CFLAGS = -g -I$(shell $(WASMER_DIR)/bin/wasmer config --includedir)
LDFLAGS = -Wl,-rpath,$(shell $(WASMER_DIR)/bin/wasmer config --libdir)
LDLIBS = $(shell $(WASMER_DIR)/bin/wasmer config --libs)

.SILENT: instance instance.o
instance: instance.o

.PHONY: clean
.SILENT: clean
clean:
	rm -f instance.o instance