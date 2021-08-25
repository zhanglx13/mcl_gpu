#! /usr/bin/env bash

##
## $1
## max perf mode: 1
## low power mode: 0
##
if [[ $1 -eq 1 ]]; then
    nvpmodel -m 0
    jetson_clocks --store
    jetson_clocks
    echo 255 > /sys/devices/pwm-fan/target_pwm
else
    nvpmodel -m 3
    jetson_clocks --restore
    echo 0 > /sys/devices/pwm-fan/target_pwm
fi
