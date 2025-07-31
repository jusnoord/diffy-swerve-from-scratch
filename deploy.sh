#!/bin/bash

DIR=/home/door/code/diffy-linux/build
cd $DIR
pwd
cmake ..
make
ssh -i ~/.ssh/robots mackwell@mackwell-K800.local "pkill DiffySwerve"
scp -i ~/.ssh/robots ./DiffySwerve mackwell@mackwell-K800.local:~/Documents
ssh -i ~/.ssh/robots mackwell@mackwell-K800.local "~/Documents/DiffySwerve"