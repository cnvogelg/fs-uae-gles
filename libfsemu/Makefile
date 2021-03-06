uname := $(shell uname -a)
ifneq ($(findstring Msys,$(uname)),)
    os = windows
else ifneq ($(findstring Darwin,$(uname)),)
    os = macosx
else
    os = linux
endif

android := 0
pkg_config = pkg-config
sdl_config = sdl-config

ifeq ($(android), 1)
os = android
cc = arm-linux-androideabi-gcc
cxx = arm-linux-androideabi-g++
ar = arm-linux-androideabi-ar
else ifeq ($(os), windows)
cc = gcc
cxx = g++
ar = ar
else ifeq ($(os), macosx)
cc = gcc
cxx = g++
ar = ar
else
cc = $(CC)
cxx = $(CXX)
ar = $(AR)
endif

# ----- GLES -----
ifeq ($(gles),1)
EXTRA_OBJ = obj/eglport.o
endif

arch :=
debug := 0
noflags := 0

warnings = -Wall
errors = -Werror=implicit-function-declaration
cppflags = $(CXXFLAGS)
cflags = -std=c99 $(warnings) $(errors) -Iinclude \
		`$(sdl_config) --cflags` `$(pkg_config) --cflags glib-2.0` \
		$(CFLAGS) -D_FILE_OFFSET_BITS=64
objects = obj/emu_emu.o obj/emu_video.o obj/emu_audio.o obj/emu_input.o \
		obj/emu_menu.o obj/emu_texture.o obj/emu_font.o \
		obj/emu_render.o obj/emu_dialog.o obj/emu_hud.o obj/emu_actions.o \
		obj/emu_shader.o obj/emu_util.o obj/emu_netplay.o \
		obj/emu_scanlines.o obj/emu_theme.o \
		\
		obj/ml_ml.o obj/ml_windows.o obj/ml_linux.o obj/ml_render.o \
		obj/ml_sdl.o obj/ml_rawinput.o obj/ml_macosx.o \
		$(EXTRA_OBJ) \
		\
		obj/base.o obj/log.o obj/config.o obj/image.o obj/ref.o \
		obj/glee.o obj/opengl.o obj/thread.o obj/string.o obj/filesys.o \
		obj/unicode.o obj/init.o obj/random.o obj/list.o obj/eventlog.o

ldflags = $(LDFLAGS)
libs =

ifeq ($(debug), 1)
	cflags += -g -O0 -fno-inline
else ifneq ($(noflags), 1)
	cflags += -g -O2
endif

ifeq ($(os), windows)

cppflags += -DWINDOWS -DWIN32
cppflags += -DWITH_SDL_VIDEO -DWITH_SDL_INPUT -DWITH_OPENAL
library = libfsemu.a

else ifeq ($(os), macosx)

uname_m := $(shell uname -m)
ifneq ($(arch),)
else ifneq ($(findstring i386,$(uname_m)),)
arch = i386
else ifneq ($(findstring Power,$(uname_m)),)
arch = ppc
else
arch = x86_64
endif
cflags += -arch $(arch)
cxxflags += -arch $(arch)
ldflags += -arch $(arch)
cppflags += -DMACOSX
cppflags += -I/System/Library/Frameworks/OpenAL.framework/Headers
cppflags += -DWITH_SDL_VIDEO -DWITH_SDL_INPUT -DWITH_OPENAL
library = libfsemu.a

else

cppflags += -DLINUX
cppflags += -DWITH_SDL_VIDEO -DWITH_SDL_INPUT -DWITH_OPENAL
cppflags += $(EXTRA_CPPFLAGS)
library = libfsemu.a
shared = libfsemu.so.1.0
shared_soname = libfsemu.so.1
shared_linkname = libfsemu.so
shared_ldflags += -shared -Wl,--as-needed -Wl,-soname=$(library_soname)
#libs += `sdl-config --libs` `pkg-config --libs glib-2.0` -lGL

# ----- GLES -----
ifeq ($(gles),1)
cppflags += -DHAVE_GLES $(GLES_INCLUDES) $(GLUES_INCLUDES)
endif

endif

all: libfsemu

libfsemu: out/$(library)

obj/%.o: src/%.c
	$(cc) $(cppflags) $(cflags) -c $< -o $@

obj/emu_%.o: src/emu/%.c
	$(cc) $(cppflags) $(cflags) -c $< -o $@

obj/ml_%.o: src/ml/%.c
	$(cc) $(cppflags) $(cflags) -c $< -o $@

out/$(library): $(objects)
ifeq ($(os), macosx)
	rm -f $@
endif
	$(ar) cru $@ $(objects)
ifeq ($(os), macosx)
	ranlib $@
endif

out/$(shared): $(objects)
	$(cc) $(shared_ldflags) $(objects) $(libs) -o $@

fs-test:
	make -C test

clean:
	rm -f obj/*.o
	rm -f out/*.a
	rm -f out/*.so*
	#make -C test clean

prefix := /usr

install:
	mkdir -p $(DESTDIR)$(prefix)/include
	cp -a include/* $(DESTDIR)$(prefix)/include/
	mkdir -p $(DESTDIR)$(prefix)/lib
	cp -a out/$(shared) $(DESTDIR)$(prefix)/lib/
	cp -a out/$(library) $(DESTDIR)$(prefix)/lib/
	ln -s $(shared) $(DESTDIR)$(prefix)/lib/$(shared_linkname)
	mkdir -p $(DESTDIR)$(prefix)/lib/pkgconfig/
	cp -a *.pc $(DESTDIR)$(prefix)/lib/pkgconfig/

