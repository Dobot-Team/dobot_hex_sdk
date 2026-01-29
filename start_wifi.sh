#!/bin/bash
export SUDO_ASKPASS=/home/robot/.config/autostart/scripts/passwd.sh
ps -A | grep hostapd | awk '{print $1}' | xargs sudo kill -9
ps -A | grep dhcpd | awk '{print $1}' | xargs sudo kill -9
sleep 2
sudo nmcli radio wifi off
sleep 2
sudo nmcli radio wifi on
sudo rfkill unblock wlan
sleep 2
nmcli device wifi list > /dev/null
sleep 2
sudo nmcli device wifi connect $1  password $2
echo "done."
