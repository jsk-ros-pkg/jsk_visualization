#!/usr/bin/env shell

# aptitude install spacenavd libspnav-dev libspnav0
sudo pkill spacenav ;
sudo service spacenavd start;
sleep 5 ;
rosrun spacenav_node spacenav_node ;
