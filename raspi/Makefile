# Makefile for FS-UAE on the Raspberry Pi

NAME=fs-uae
SRC_BIN=../out/$(NAME)
EXTRA_DIR=../share
BUILD_DIR=build
STRIP=$(COMPILER_PREFIX)strip
S2_TARGET=rpi

.phony: all cross cross-build cross-link local clean

all: cross

# cross build is done with scratchbox2
# (http://maemo.gitorious.org/scratchbox2/scratchbox2)
#
# It is assumed that on the chroot filesystem the following files are available:
#
# in $(CHROOT_DIR)/home/user:
#   # fetch and build from https://github.com/cnvogelg/glues
#   opt/glues/include/glu.h     
#   opt/glues/lib/libGLUES.a
#   opt/vc                  <- tree copy from /opt/vc (no link!)
#   usr/bin/pkg-config      <- copy from /usr/bin (no link!)
#   usr/bin/sdl-config      <- copy from /usr/bin (no link!)
#
cross: cross-build cross-link
# build runs through compile but currently fails on link line... so ignore it and...
cross-build:
	cd .. && sb2 -t $(S2_TARGET) $(MAKE) raspi=1 COMPILER_PREFIX=/usr/bin/ RASPI_ROOT=/home/user/ || exit 0
# ...workaround for failing link step: re-run in emulation mode
cross-link:
	cd .. && sb2 -t $(S2_TARGET) -m emulate $(MAKE) raspi=1 COMPILER_PREFIX=/usr/bin/ RASPI_ROOT=/home/user/

# local build on raspi
local:
	$(MAKE) -C .. raspi=1

clean:
	$(MAKE) -C .. raspi=1 clean


