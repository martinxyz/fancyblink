#!/bin/bash
make clean
make lib
python fancyblink_lutgen.py
make fancyblink.elf
make fancyblink.bin

