#!/usr/bin/bash

GET_PROP_ATZ=$(getprop persist.sys.timezone)
if [ -f "/data/params/d/DPTimeZone" ]; then
    GET_PROP_STZ=$(cat /data/params/d/DPTimeZone)
fi

if [ "$GET_PROP_STZ" != "" ] && [ "$GET_PROP_ATZ" != "$GET_PROP_STZ" ]; then
    setprop persist.sys.timezone $GET_PROP_STZ
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

