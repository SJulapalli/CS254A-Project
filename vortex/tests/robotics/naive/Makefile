ROOT_DIR := $(realpath ../../..)
include $(ROOT_DIR)/config.mk

PROJECT := naive

SRC_DIR := $(VORTEX_HOME)/tests/robotics/$(PROJECT)

SRCS := $(SRC_DIR)/main.cpp

VX_SRCS := $(SRC_DIR)/kernel.cpp

OPTS ?= -f $(SRC_DIR)/../test_data/data_2000.bin

KERNEL_LIB := vortex2

include ../common.mk