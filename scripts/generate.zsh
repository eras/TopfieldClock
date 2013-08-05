#!/usr/bin/zsh
for a in "00100000 01110101 01100111 0S"{0..9}"      111110S"{0..9}"      111 10S"{0..9}"      1111110 S"{0..9}"       11000111 2 011000000 2"; do
    echo $a
done | shuf | while read a; do
    echo "$a" | ./send-sequence /dev/ttyUSB1
done
