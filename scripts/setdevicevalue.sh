#!/bin/bash

mcPort='/dev/ttyUSB1';

obj_dst=1;
obj_src=3;
message_type=2;

jsonValue="{\"ssn\":{\"v\":1,\"obj\":1,\"cmd\":\"sdv\", \"data\": {\"adev\":1006,\"acmd\":1,\"aval\":12510}}}";
len=${#jsonValue};

crc=$(./crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x\r" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;

echo cmd transferred
