#!/bin/sh
sudo nmap -sP 192.168.0.0-254 | grep -B 2 -i -E "40:61:86:04:8D:6(A|B)" | grep "Nmap scan report" | awk '{print "talos ip: " $5}'

