#!/bin/bash

#jsonFile='release/loadprefs_demo2.json';
jsonFile='test/loadprefs2.ini';
mcPort='/dev/ttyUSB1';

obj_dst=1;
obj_src=3;
message_type=7;

jsonValue=$(<$jsonFile)
len=${#jsonValue};

crc=$(./crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x\r" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;
#cat $jsonFile  >> $mcPort
#printf "%04x" "$crc" >> $mcPort;
#printf "\0" >> $mcPort;

echo File transferred
