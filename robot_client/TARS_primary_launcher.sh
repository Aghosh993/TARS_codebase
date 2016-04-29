#!/bin/bash
IPADDR=172.16.24.10
echo "Launching Primary Camera stream..."
./picam_stream.sh $IPADDR &
exit 0
