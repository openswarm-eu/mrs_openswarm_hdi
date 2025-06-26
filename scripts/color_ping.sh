#!/bin/bash

host=$1
threshold1=50    # Green if < 50ms
threshold2=200   # Yellow if < 200ms, else Red

while read -r line; do
    if [[ $line =~ time=([0-9.]+)\ ms ]]; then
        time=${BASH_REMATCH[1]}
        if (( $(echo "$time < $threshold1" | bc -l) )); then
            color="\e[32m" # Green
        elif (( $(echo "$time < $threshold2" | bc -l) )); then
            color="\e[33m" # Yellow
        else
            color="\e[31m" # Red
        fi
        echo -e "${line/time=/$color time=}\e[0m"
    else
        echo "$line"
    fi
done < <(ping "$host")
