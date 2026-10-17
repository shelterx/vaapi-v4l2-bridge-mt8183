CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
TARGETS := v4l2_fmt_test v4l2_mdp3_stream

all: $(TARGETS)

v4l2_fmt_test: v4l2_fmt_test.c
	$(CC) $(CFLAGS) -o $@ $<

v4l2_mdp3_stream: v4l2_mdp3_stream.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGETS)

.PHONY: all clean
