#!/bin/bash

#This script is launched automatically in the BeagleBone Black
#on every boot and loads everything that the board needs to
#act as an autopilot. The COPY_CAPES environment variable copies
#the capies on every boot if set to 1.
#
# Victor Mayoral Vilches - Erle Robotics [victor@erlerobot.com]

# Cape source location
LOCATION="/root/ardupilot/Tools/Linux_HAL_Essentials"

# Copy capes
COPY_CAPES=0

if (($COPY_CAPES == 1)); then 
    cp $LOCATION/BB-SPI0-PXF-01-00A0.dtbo /lib/firmware/
    cp $LOCATION/BB-SPI1-PXF-01-00A0.dtbo /lib/firmware/
    cp $LOCATION/BB-BONE-PRU-05-00A0.dtbo /lib/firmware/
    cp $LOCATION/rcinpru0 /lib/firmware
    cp $LOCATION/pwmpru1 /lib/firmware
fi 

# Loading the capes
echo BB-BONE-PRU-05 > /sys/devices/bone_capemgr.*/slots
echo BB-SPI0-PXF-01 > /sys/devices/bone_capemgr.*/slots
echo BB-SPI1-PXF-01 > /sys/devices/bone_capemgr.*/slots
echo BB-UART5 > /sys/devices/bone_capemgr.*/slots
echo BB-UART4 > /sys/devices/bone_capemgr.*/slots
echo BB-UART2 > /sys/devices/bone_capemgr.*/slots
echo am33xx_pwm > /sys/devices/bone_capemgr.*/slots
echo bone_pwm_P8_36 > /sys/devices/bone_capemgr.*/slots

# Line for making PREEMPT_RT work
#echo 0:rcinpru0 > /sys/devices/ocp.3/4a300000.prurproc/load

# Logging
dmesg | grep "SPI"
dmesg | grep "PRU"
cat /sys/devices/bone_capemgr.*/slots

# Force the ip address of the hotspot
ifconfig wlan2 11.0.0.1

# Give the system time to load all the capes
#   experienced has proved that generally the buzzer needs some time
sleep 1

cd /root
(
    date
    init 3
    #killall -q udhcpd
    while :; do
    # Set CPU at max speed
    cpufreq-set -f 1000MHz
    # Start copter, modify if other vehicle is needed
    ./ArduCopter.elf -A udp:192.168.7.1:6000 -B /dev/ttyO5
    #./ArduCopter.elf -A udp:11.0.0.2:6000 -B /dev/ttyO5
        #./ArduCopter.elf -A /dev/ttyO0 -B /dev/ttyO5
    #./ArduCopter.elf -A tcp:*:6000:wait -B /dev/ttyO5
    done
) >> ~/copter.log 2>&1
