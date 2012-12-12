uname := $(shell uname -a)
arch :=

android := 0

pkg_config = pkg-config
sdl_config = sdl-config

ifeq ($(android), 1)

os = android
cc = arm-linux-androideabi-gcc
cxx = arm-linux-androideabi-g++
ar = arm-linux-androideabi-ar
make = make

# ----- open pandora -----
else ifeq ($(pandora), 1)
os = pandora
cc = pandora-gcc
cxx = pandora-g++
ar = pandora-ar
pkg_config = pandora-pkg-config
make = make

# ----- raspberry pi -----
else ifeq ($(raspi), 1)
os = raspi
cc = $(COMPILER_PREFIX)gcc
cxx = $(COMPILER_PREFIX)g++
ar = $(COMPILER_PREFIX)ar
pkg_config = $(RASPI_ROOT)/usr/bin/pkg-config
sdl_config = $(RASPI_ROOT)/usr/bin/sdl-config
make = make
# ----- windows -----
else ifneq ($(findstring Msys,$(uname)),)

os = windows
cc = gcc
cxx = g++
ar = ar
make = make

else ifneq ($(findstring Darwin,$(uname)),)

os = macosx
cc = gcc
cxx = g++
ar = ar
make = make

else ifneq ($(findstring FreeBSD,$(uname)),)

os = freebsd
cc = $(CC)
cxx = $(CXX)
ar = $(AR)
make = gmake

else ifneq ($(findstring OpenBSD,$(uname)),)

os = openbsd
cc = $(CC)
cxx = $(CXX)
ar = $(AR)
make = gmake

else
os = linux
cc = $(CC)
cxx = $(CXX)
ar = $(AR)
make = make

endif

debug := 0
devel := 0
optimize := 1
noflags := 0
prefix := /usr
docdir := $(prefix)/share/doc/fs-uae
