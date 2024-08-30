sudo sh -c "echo 1 >/proc/sys/net/ipv4/ip_forward"
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"
sudo ufw disable

#sudo service procps restart
#netstat -g

roslaunch mission_planner multimaster.launch __ns:=uav$UAV_ID
