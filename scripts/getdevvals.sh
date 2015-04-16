#!/bin/bash

jsonFile='release/getdevvals.json';
#jsonFile='test2/getdevvals.json';
mcPort='/dev/ttyUSB0';

obj_dst=1;
obj_src=3;
message_type=2;

jsonValue=$(<$jsonFile)
len=${#jsonValue};

crc=$(./crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;

echo File transferred
