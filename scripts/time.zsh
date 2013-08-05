#!/usr/bin/zsh
while true; do
    set `date +%H%M | sed 's/\([0-9]\)/\1 /g'`
    echo "00100000 01110101 01100111 0S$1      111110S$2      111 10S$3      1111110 S$4       11000111 2 011000000 2" | ./send-sequence /dev/ttyUSB1
    sleep $(( 61 - $(date +%S) ))
done
