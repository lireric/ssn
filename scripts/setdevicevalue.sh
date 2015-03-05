#!/bin/bash

mcPort='/dev/ttyUSB0';

obj_dst=1;
obj_src=3;
message_type=2;

jsonValue="{\"ssn\":{\"v\":1,\"obj\":%u,\"cmd\":\"sdv\", \"data\": {\"adev\":4004,\"acmd\":1,\"aval\":1}}}";
len=${#jsonValue};

crc=$(./crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x\r" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;

echo cmd transferred
