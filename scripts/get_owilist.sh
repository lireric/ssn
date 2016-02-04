#!/bin/bash

jsonFile='release/get_owilist.json';
mcPort='/dev/ttyUSB1';
#mcPort='/home/eric/qqq.txt';

obj_dst=1;
obj_src=3;
message_type=2;

jsonValue=$(<$jsonFile)
len=${#jsonValue};

crc=$(./crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x\r" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;

echo File transferred
