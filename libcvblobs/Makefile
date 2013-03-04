all: cvblobs

TARBALL = build/cvblobs8.3_linux.tgz
TARBALL_URL = http://farnsworth.csres.utexas.edu/share/cvblobs8.3_linux.tgz
SOURCE_DIR = build/cvblobs8.3_linux
MD5SUM_FILE = cvblobs8.3_linux.tgz.md5sum
UNPACK_CMD = tar xzf
TARBALL_PATCH = cvblobs.so.patch
include $(shell rospack find mk)/download_unpack_build.mk

cvblobs: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && make
	mkdir -p include/cvblobs
	cp $(SOURCE_DIR)/*.h include/cvblobs/
	mkdir -p lib
	cp $(SOURCE_DIR)/*.so lib/
	touch cvblobs

clean:
	-rm -rf src $(SOURCE_DIR) cvblobs include lib

wipe: clean
	-rm -rf build
