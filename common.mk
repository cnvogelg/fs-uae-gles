uname := $(shell uname -a)
arch :=

android := 0
pandora := 1

pkg_config = pkg-config
ifeq ($(android), 1)
os = android
cc = arm-linux-androideabi-gcc
cxx = arm-linux-androideabi-g++
ar = arm-linux-androideabi-ar
make = make
else ifeq ($(pandora), 1)
os = pandora
cc = pandora-gcc
cxx = pandora-g++
ar = pandora-ar
pkg_config = pandora-pkg-config
make = make
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
else
os = linux
cc = $(CC)
cxx = $(CXX)
ar = $(AR)
make = make
endif

debug := 0
optimize := 1
noflags := 0
prefix := /usr
docdir := $(prefix)/share/doc/fs-uae
