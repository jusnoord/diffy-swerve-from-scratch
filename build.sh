#!/bin/bash


DIR=/home/door/code/diffy-linux/build
cd $DIR
pwd
cmake ..
if [ $? -ne 0 ]; then
    echo "Error: cmake failed."
    exit 1
fi
make
if [ $? -ne 0 ]; then
    echo "Error: make failed."
    exit 1
fi