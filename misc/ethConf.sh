#!/bin/bash

# jumbo frame
sudo ip link set eth0 down
sudo ip link set eth0 mtu 9000
sudo ip link set eth0 up

# receive buffer size
sudo ethtool -G eth0 rx 4096 

# socket buffer to 32mb
sudo sh -c "echo 'net.core.rmem_default=33554432' >> /etc/sysctl.conf"
sudo sh -c "echo 'net.core.rmem_max=33554432' >> /etc/sysctl.conf"
sudo sysctl -p

# proper eth addr
sudo ip addr flush dev eth0
sudo ip addr add 192.168.0.1/24 dev eth0
