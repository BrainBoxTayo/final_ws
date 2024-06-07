#!/usr/bin/bash

# script that checks if the esp is bound, releases the old bind and binds it again
if [ -e "/dev/rfcomm1" ]; then
 sudo rfcomm release rfcomm1
fi

sudo rfcomm bind 1 D4:8A:FC:A8:96:7A 
