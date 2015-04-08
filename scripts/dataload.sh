#!/bin/bash

jsonFile='test/getdevvals.json';
mcPort='/dev/ttyUSB0';

ssn_acc=1;
obj_dst=1;
obj_src=3;
url="http://localhost/ssn/index.php";
message_type=2;

#jsonValue=$(<$jsonFile)
jsonValue="{\"ssn\":{\"json_file_name\":\"$jsonFile\",\"com_port_name\":\"$mcPort\",\"com_port_speed\":57600,\"obj_dst\":$obj_dst,\"obj_src\":$obj_src}}";

len=${#jsonValue};

msg_data=$(./ssn "$jsonValue");

curl --header "ssn-acc: $ssn_acc"  --header "ssn-obj: $obj_dst" --header "ssn-aes128: 0" --header "ssn-base64: 0" -X POST $url --data "$msg_data"
