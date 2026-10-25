# KAC -- build (gcc, C99). Products land in build/ and nothing is generated
# beside the sources.
CC       ?= gcc
CFLAGS   ?= -std=c99 -Wall -Wextra -pedantic-errors -O3
CPPFLAGS  = -I.
LDLIBS    = -lm

BUILD = build

# The PATH carries clang-format under its versioned name, so that is the default.
CLANG_FORMAT ?= clang-format-18
FMT_SRC = kac.c kac_encode.c kac_decode.c kac.h kac_internal.h kac_dqt.h wav_io.h \
          tests/test_roundtrip.c tests/test_loop.c tests/test_bad.c \
          tests/test_seek.c tests/test_select.c tests/test_block.c \
          tests/test_stream.c tests/kac_test_util.h tools/calibrate_v.c

HDR   = kac_internal.h kac_dqt.h kac.h
OBJ   = $(BUILD)/kac_encode.o $(BUILD)/kac_decode.o
TESTS = $(BUILD)/test_roundtrip $(BUILD)/test_loop $(BUILD)/test_bad \
        $(BUILD)/test_seek $(BUILD)/test_select $(BUILD)/test_block \
        $(BUILD)/test_stream

all: $(BUILD)/kac $(TESTS)

$(BUILD):
	mkdir -p $@

$(BUILD)/kac_encode.o: kac_encode.c kac.h kac_internal.h kac_dqt.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c kac_encode.c -o $@

$(BUILD)/kac_decode.o: kac_decode.c kac.h kac_internal.h kac_dqt.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c kac_decode.c -o $@

$(BUILD)/kac: kac.c $(OBJ) $(HDR) wav_io.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ kac.c $(OBJ) $(LDLIBS)

$(BUILD)/test_%: tests/test_%.c tests/kac_test_util.h $(OBJ) $(HDR) | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(OBJ) $(LDLIBS)

dqt:
	python3 tools/gen_dqt.py kac_dqt.h

format:
	$(CLANG_FORMAT) -i $(FMT_SRC)

format-check:
	$(CLANG_FORMAT) --dry-run --Werror $(FMT_SRC)

check: all
	bash tests/run.sh

clean:
	rm -rf $(BUILD)

.PHONY: all dqt format format-check check clean
