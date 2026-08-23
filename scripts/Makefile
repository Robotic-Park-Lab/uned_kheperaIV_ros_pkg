# Build the onboard C programs that run on the Khepera IV's own embedded
# Linux (server.c, prog-template.c) against libkhepera.
#
# libkhepera is K-Team SA's proprietary SDK for the Khepera platform -- it
# is NOT vendored in this repo (no license file grants redistribution, see
# AUDIT.md on the doc branch and the README's "libkhepera" section). Obtain
# it separately (from K-Team, or your own existing copy) and point
# LIBKHEPERA_PATH at its build output directory -- the one containing
# include/ and lib/ (e.g. libkhepera-2.1/build-khepera-<kernel-version>).
#
# Native build, for compile-checking on a dev machine (won't run on a real
# Khepera without the actual hardware/drivers):
#   make LIBKHEPERA_PATH=/path/to/libkhepera-2.1/build-khepera-3.18.18-custom
#
# Cross-compile for the Khepera's ARM target (requires the Poky/Yocto ARM
# toolchain the Khepera IV ships with, also not part of this repo -- see
# libkhepera's own template/Makefile for the full toolchain environment
# variables if cross-compiling from scratch):
#   make CC=arm-poky-linux-gnueabi-gcc LIBKHEPERA_PATH=/path/to/build-khepera-3.18.18-custom

ifndef LIBKHEPERA_PATH
$(error LIBKHEPERA_PATH is not set -- point it at libkhepera's build-khepera-<version> directory (the one with include/ and lib/). See the comment at the top of this Makefile)
endif

CC      ?= gcc
CFLAGS  ?= -O2 -Wall
INCS    := -I $(LIBKHEPERA_PATH)/include
LIBS    := -L $(LIBKHEPERA_PATH)/lib -lkhepera -lpthread -lm

TARGETS := server prog-template

.PHONY: all clean

all: $(TARGETS)

server: server.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $< $(LIBS)

prog-template: prog-template.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $< $(LIBS)

clean:
	rm -f $(TARGETS) *.o
