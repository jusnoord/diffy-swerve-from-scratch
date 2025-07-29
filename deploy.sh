#!/bin/bash

DIR=/home/door/code/diffy-linux/build
cd $DIR
pwd
cmake ..
make
scp -i ~/.ssh/robots ./DiffySwerve mackwell@10.93.12.200:~/Documents
