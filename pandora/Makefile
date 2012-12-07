# Makefile for FS-UAE on Open Pandora platform

VERSION=$(strip $(shell cat ../VERSION))
MKSQUASHFS=mksquashfs
NAME=fs-uae
SRC_BIN=../out/$(NAME)
PND_FILES=PXML.xml run.sh Default.fs-uae.templ
EXTRA_LIBS=$(PNDSDK)/usr/lib/libGLUES_CM.so.0 $(PNDSDK)/usr/lib/libopenal.so.1
EXTRA_DIR=../share
ICON_FILE=../share/icons/hicolor/64x64/apps/fs-uae.png
PND_OUT=fs-uae_$(VERSION).pnd
SQUASH_FS=build.squash
BUILD_DIR=build
STRIP=pandora-strip

.phony: all gather clean clean-all

all: $(PND_OUT)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

gather: $(BUILD_DIR) $(PND_FILES) $(ICON_FILE) $(SRC_BIN)
	cp $(SRC_BIN) $(BUILD_DIR)/
	cp -a $(PND_FILES) $(BUILD_DIR)/
	cp $(ICON_FILE) $(BUILD_DIR)/
	cp $(EXTRA_LIBS) $(BUILD_DIR)/
	cp -r $(EXTRA_DIR) $(BUILD_DIR)/
	$(STRIP) $(BUILD_DIR)/$(NAME)

$(SQUASH_FS): gather
	rm -f $(SQUASH_FS)
	$(MKSQUASHFS) $(BUILD_DIR) $(SQUASH_FS)

$(PND_OUT): $(SQUASH_FS)
	cat $(SQUASH_FS) PXML.xml $(ICON_FILE) > $(PND_OUT)

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(SQUASH_FS) $(PND_OUT)

clean-all: clean
	$(MAKE) -C .. pandora=1 clean

$(FS_UAE_BIN):
	$(MAKE) -C .. pandora=1
