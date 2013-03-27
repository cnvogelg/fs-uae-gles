# Makefile for FS-UAE on a generic OpenGL ES target

all: fs-uae

fs-uae:
	cd .. && $(MAKE) gles=1 \
		CC=$(COMPILER_PREFIX)gcc CXX=$(COMPILER_PREFIX)g++ AR=$(COMPILER_PREFIX)ar \
		sdl_config=$(TARGET_ROOT)/usr/bin/sdl-config \
		pkg_config=$(TARGET_ROOT)/usr/bin/pkg-config \
		EXTRA_CPPFLAGS="-DUSE_EGL_SDL -DUSE_GLES1" \
		GLES_INCLUDES="" \
		GLES_LIBS="$(SB2_LDFLAGS) -lEGL -lGLESv1_CM"

clean:
	$(MAKE) -C .. clean


