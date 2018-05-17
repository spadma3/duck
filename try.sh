#!/bin/bash

PIPE=/tmp/catpipe
trap "rm -f $PIPE" exit 1
[[ ! -p $PIPE ]] && mkfifo $PIPE

while true; do
     while read line; do
          case "$line" in
               @exit) rm -f $PIPE && exit 0;;
               @*) eval "${line#@}" ;;
               * ) echo "$line" ;;
          esac
     done <$PIPE
done

exit 2
