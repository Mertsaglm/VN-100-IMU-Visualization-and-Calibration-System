# VN-100 portable core — PC build
# Proof of the "same driver on PC + STM32" principle: the HAL-free Core/ files
# build against the PC port. Usage:  make            -> host_selftest + vn100_cli
#                                    make selftest   -> build+run the self-test
#                                    make clean
CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -O2 -I../Core/Inc
LDFLAGS ?= -lm

# Platform-INDEPENDENT core (SHARED with STM32) — no HAL
CORE := ../Core/Src/vn100_protocol.c ../Core/Src/vn100_binary.c ../Core/Src/vn100.c

BUILD := build

all: $(BUILD)/host_selftest $(BUILD)/vn100_cli

$(BUILD):
	mkdir -p $(BUILD)

# Self-test: verifies the protocol + dual-mode core + host_link (VN MODE relay)
$(BUILD)/host_selftest: host_selftest.c $(CORE) ../Core/Src/host_link.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# CLI: console app that runs the same core through the PC port (vn100_port_pc.c)
$(BUILD)/vn100_cli: vn100_cli.c vn100_port_pc.c $(CORE) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

selftest: $(BUILD)/host_selftest
	$(BUILD)/host_selftest

clean:
	rm -rf $(BUILD)

.PHONY: all selftest clean
