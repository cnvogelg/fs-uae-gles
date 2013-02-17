# Makefile for FS-UAE on the Pandaboard

.phony: all cross fs-uae clean

all: fs-uae

cross:
	@echo "TODO"

fs-uae:
	cd .. && $(MAKE) gles=1 \
		CC=$(COMPILER_PREFIX)gcc CXX=$(COMPILER_PREFIX)g++ AR=$(COMPILER_PREFIX)ar \
		sdl_config=$(PANDA_ROOT)/usr/bin/sdl-config \
		pkg_config=$(PANDA_ROOT)/usr/bin/pkg-config \
		EXTRA_CPPFLAGS="-DUSE_EGL_SDL -DUSE_GLES1" \
		GLES_INCLUDES="-I$(PANDA_ROOT)/usr/lib/pvr-omap4-egl/include" \
		GLES_LIBS="-L$(PANDA_ROOT)/usr/lib/pvr-omap4-egl -lEGL -lGLESv1_CM"

clean:
	$(MAKE) -C .. clean


