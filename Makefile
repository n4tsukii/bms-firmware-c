CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wstrict-prototypes -Werror -O2 -Iinclude
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer -g

FW_SRC  := src/bms.c src/bms_can.c src/can.c
SIM_SRC := $(FW_SRC) src/main.c hal/hal_sim.c

.PHONY: all test test-asan sim clean

all: bms_sim

bms_sim: $(SIM_SRC) include/*.h
	$(CC) $(CFLAGS) $(SIM_SRC) -o $@

test_runner: $(FW_SRC) tests/test_main.c include/*.h
	$(CC) $(CFLAGS) $(FW_SRC) tests/test_main.c -o $@

test: test_runner
	./test_runner

# AddressSanitizer + UBSan (Linux / WSL / macOS)
test-asan: $(FW_SRC) tests/test_main.c include/*.h
	$(CC) $(CFLAGS) $(SAN) $(FW_SRC) tests/test_main.c -o test_runner_asan
	./test_runner_asan

sim: bms_sim
	./bms_sim overtemp 15 bms_can.log

clean:
	$(RM) bms_sim test_runner test_runner_asan *.exe bms_can.log
