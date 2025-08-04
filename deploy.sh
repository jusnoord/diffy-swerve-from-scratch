#!/bin/bash

DNS_ADDR="mackwell-K800.local"
IP_ADDR="10.93.12.201"
TARGET=""

echo "Trying to resolve target address..."

ping -c 1 $DNS_ADDR &> /dev/null
if [ $? -eq 0 ]; then
    TARGET=$DNS_ADDR
    echo "Resolved target: $TARGET"
else
    ping -c 1 $IP_ADDR &> /dev/null
    if [ $? -eq 0 ]; then
        TARGET=$IP_ADDR
        echo "Resolved target: $TARGET"
    else
        echo "Error: Could not resolve target address (DNS or IP unreachable)."
        exit 1
    fi
fi

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

ssh -i ~/.ssh/robots mackwell@$TARGET "pkill DiffySwerve"
if [ $? -ne 0 ]; then
    echo "Error: SSH pkill failed."
    exit 1
fi

scp -i ~/.ssh/robots ./DiffySwerve mackwell@$TARGET:~/Documents
if [ $? -ne 0 ]; then
    echo "Error: SCP failed."
    exit 1
fi

ssh -i ~/.ssh/robots mackwell@$TARGET "~/Documents/DiffySwerve"
if [ $? -ne 0 ]; then
    echo "Error: SSH run failed."
    exit 1
fi