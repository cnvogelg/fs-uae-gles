# Makefile for FS-UAE on Open Pandora platform

PATCHLEVEL=1
VERSION=$(strip $(shell cat ../VERSION))
VER_MAJ=$(strip $(shell cat ../VERSION | cut -f1 -d.))
VER_MIN=$(strip $(shell cat ../VERSION | cut -f2 -d.))
VER_REL=$(strip $(shell cat ../VERSION | cut -f3 -d.))

MKSQUASHFS=mksquashfs
NAME=fs-uae
SRC_BIN=../$(NAME)
PND_FILES=run.sh Default.fs-uae.templ
EXTRA_LIBS=$(PNDSDK)/usr/lib/libopenal.so.1
EXTRA_DIR=../share
ICON_FILE=../share/icons/hicolor/64x64/apps/fs-uae.png
PND_OUT=fs-uae_$(VERSION)_p$(PATCHLEVEL).pnd
SQUASH_FS=build.squash
BUILD_DIR=build
STRIP=pandora-strip

PANDORA_CFLAGS := -march=armv7-a -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp
PANDORA_CFLAGS += -msoft-float -ffast-math -fomit-frame-pointer -fstrict-aliasing
PANDORA_CFLAGS += -mstructure-size-boundary=32 -fexpensive-optimizations 
PANDORA_CFLAGS += -fweb -frename-registers -falign-functions=16 -falign-loops -falign-labels -falign-jumps
PANDORA_CFLAGS += -finline -finline-functions -fno-common -fno-builtin -fsingle-precision-constant

.phony: all gather genpxml clean clean-all fs-uae strip run

all: $(PND_OUT)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

gather: strip $(BUILD_DIR) $(PND_FILES) $(ICON_FILE) $(SRC_BIN)
	cp $(SRC_BIN) $(BUILD_DIR)/
	cp -a $(PND_FILES) $(BUILD_DIR)/
	cp $(ICON_FILE) $(BUILD_DIR)/
	cp $(EXTRA_LIBS) $(BUILD_DIR)/
	cp -r $(EXTRA_DIR) $(BUILD_DIR)/

genpxml: PXML.xml $(BUILD_DIR)
	sed -e 's/@VERSION/$(VERSION)/' \
		-e 's/@PATCHLEVEL/$(PATCHLEVEL)/' \
		-e 's/@VER_MAJ/$(VER_MAJ)/' \
		-e 's/@VER_MIN/$(VER_MIN)/' \
		-e 's/@VER_REL/$(VER_REL)/' \
		< PXML.xml > $(BUILD_DIR)/PXML.xml

$(SQUASH_FS): gather genpxml
	rm -f $(SQUASH_FS)
	$(MKSQUASHFS) $(BUILD_DIR) $(SQUASH_FS)

$(PND_OUT): $(SQUASH_FS)
	cat $(SQUASH_FS) $(BUILD_DIR)/PXML.xml $(ICON_FILE) > $(PND_OUT)

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SQUASH_FS) $(PND_OUT)

clean-all: clean
	$(MAKE) -C .. clean

strip: fs-uae
	$(STRIP) $(SRC_BIN)
	
fs-uae:
	$(MAKE) -C .. gles=1 \
		CC=pandora-gcc CXX=pandora-g++ AR=pandora-ar \
		pkg_config=pandora-pkg-config \
		sdl_config=$(PNDSDK)/usr/bin/sdl-config \
		EXTRA_CPPFLAGS="$(PANDORA_CFLAGS) -DUSE_EGL_SDL -DUSE_GLES1 -DPANDORA" \
		GLES_INCLUDES=-I$(PNDSDK)/usr/include \
		GLES_LIBS="-lEGL -lGLES_CM" \
		SYS_INCLUDES=-I$(PNDSDK)/usr/include

	