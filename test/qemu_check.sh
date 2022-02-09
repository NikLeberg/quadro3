#!/usr/bin/env bash

set -e

# Check the output of a qemu run in the qemu_log.txt file.

# These lines should not to be in the output.
if ! test $(grep -c "ets Jul 29 2019" qemu_log.txt) -eq 1
then
    echo "FAIL: No ROM boot message present. Did QEMU boot correctly?"
    exit 1
fi
if ! test $(grep -c "Running all qemu enabled tests." qemu_log.txt) -eq 1
then
    echo "FAIL: app_main() was not executed. Did QEMU boot correctly?"
    exit 1
fi

# These lines should not be in the output.
if ! test $(grep -c "FAIL" qemu_log.txt) -eq 0
then
    echo "FAIL: One or more of the tests failed."
    exit 1
fi

# Decode core panic logs.
if ! test $(grep -c "Guru Meditation Error" qemu_log.txt) -eq 0
then
    echo "FAIL: Core paniced."
    cat qemu_log.txt | sed -ne '/Backtrace/,$ p' | grep -oh "0x4......." > addresses.txt
    xtensa-esp32-elf-addr2line -aipfe test/build/quadro3_test.elf < addresses.txt | tee -a qemu_log.txt
    rm addresses.txt
    exit 1
fi
