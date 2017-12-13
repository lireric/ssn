#!/bin/bash

mcPort='/dev/ttyUSB1';

obj_dst=4;
obj_src=3;
message_type=2;

#jsonValue="{\"ssn\":{\"v\":1,\"obj\":4,\"cmd\":\"sdv\", \"data\": {\"adev\":10001,\"acmd\":0,\"aval\":\"Test\"}}}";
#jsonValue="{\"ssn\":{\"v\":1,\"obj\":4,\"cmd\":\"sdv\", \"data\": {\"adev\":6001,\"acmd\":0,\"aval\":-1000}}}";
jsonValue="{\"ssn\":{\"v\":1,\"obj\":4,\"cmd\":\"sdv\", \"data\": {\"adev\":11,\"acmd\":1,\"aval\":10002}}}";
len=${#jsonValue};

crc=$(../crc16 "$jsonValue" $len);

printf "===ssn1%04x%04x%02x%04x%s%04x\r" "$obj_dst" "$obj_src" "$message_type" "$len" "$jsonValue" "$crc" > $mcPort;

echo cmd transferred
