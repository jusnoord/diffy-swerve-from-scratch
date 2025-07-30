#!/bin/bash

DIR=/home/door/code/diffy-linux/build
cd $DIR
pwd
cmake ..
make
ssh -i ~/.ssh/robots mackwell@10.93.12.200 "pkill DiffySwerve"
scp -i ~/.ssh/robots ./DiffySwerve mackwell@10.93.12.200:~/Documents
ssh -i ~/.ssh/robots mackwell@10.93.12.200 "cd ~/Documents; ./DiffySwerve >> ./meow.txt"