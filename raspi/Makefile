# Makefile for FS-UAE on the Raspberry Pi

SB2_TARGET=rpi
SB2_RUNTIME=eval

RASPI_CFLAGS := -march=armv6 -mfloat-abi=hard -mfpu=vfp 

.phony: all cross cross-build cross-link fs-uae clean

all: fs-uae

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
	$(MAKE) fs-uae \
		COMPILER_PREFIX=/usr/bin/ \
		RASPI_ROOT=/home/user/ \
		SB2_RUNTIME="sb2 -t $(SB2_TARGET) --" \
	|| exit 0
# ...workaround for failing link step: re-run in emulation mode
cross-link:
	$(MAKE) fs-uae \
		COMPILER_PREFIX=/usr/bin/ \
		RASPI_ROOT=/home/user/ \
		SB2_RUNTIME="sb2 -t $(SB2_TARGET) -m emulate --"

# local build on raspi
fs-uae:
	cd .. && $(SB2_RUNTIME) $(MAKE) gles=1 \
		CC=$(COMPILER_PREFIX)gcc CXX=$(COMPILER_PREFIX)g++ AR=$(COMPILER_PREFIX)ar \
		sdl_config=$(RASPI_ROOT)/usr/bin/sdl-config \
		pkg_config=$(RASPI_ROOT)/usr/bin/pkg-config \
		EXTRA_CPPFLAGS=\"$(RASPI_CFLAGS) -DUSE_EGL_RAW -DUSE_GLES1 -DRPI\" \
		GLES_INCLUDES=\"-I$(RASPI_ROOT)/opt/vc/include -I$(RASPI_ROOT)/opt/vc/include/interface/vcos/pthreads\" \
		GLES_LIBS=\"-L$(RASPI_ROOT)/opt/vc/lib -lEGL -lGLESv1_CM\" \
		GLUES_INCLUDES=-I$(RASPI_ROOT)/opt/glues/include \
		GLUES_LIBS=\"-L$(RASPI_ROOT)/opt/glues/lib -lGLUES\"

clean:
	$(MAKE) -C .. clean


