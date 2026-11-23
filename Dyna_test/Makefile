# Compilation des programmes Dynamixel (AX-12A, protocole 1.0)
# Necessite DynamixelSDK C++ installe dans /usr/local :
#   Raspberry Pi (ARM)  -> libdxl_sbc_cpp.so  (build/linux_sbc)
#   PC Linux x86_64     -> libdxl_x64_cpp.so  (build/linux64)
#
#   make            compile tout dans bin/
#   make clean      supprime bin/

CXX      := g++
CXXFLAGS := -O2 -Wall -DLINUX -D_GNU_SOURCE -I/usr/local/include
LDFLAGS  := -L/usr/local/lib

ifeq ($(shell uname -m),x86_64)
  DXL_LIB := -ldxl_x64_cpp
else
  DXL_LIB := -ldxl_sbc_cpp
endif

SRCS := $(wildcard src/*.cpp)
BINS := $(patsubst src/%.cpp,bin/%,$(SRCS))

all: $(BINS)

bin/%: src/%.cpp | bin
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(DXL_LIB)

bin:
	mkdir -p bin

clean:
	rm -rf bin

.PHONY: all clean
