#!/bin/bash
url1="http://www.hivemq.com/demos/websocket-client/"
url2="https://docs.google.com/spreadsheets/d/1TVISdaHHRgOeV0VRTV_zvCYoh-BjUyAa_kw1BF3V_lw/edit#gid=0"

echo "opening $url1 and $url2 in firefox"

firefox -new-window $url1 $url2
