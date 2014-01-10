#!/bin/bash

function getWID(){
    name=$1
    hint="$2"
    THE_PID=`ps aux | grep $name | grep "$hint" | grep -v grep | awk '{print $2}'`
    THE_WINDOW_ID=`wmctrl -lp | grep $THE_PID | awk '{print $1}'`
    echo $THE_WINDOW_ID
}

sleep 6

TOP_VIEW_RVIZ_WID=`getWID rviz top_view.rviz`
wmctrl -ia $TOP_VIEW_RVIZ_WID
wmctrl -ir $TOP_VIEW_RVIZ_WID -e 0,0,0,800,900 

SIDE_VIEW_RVIZ_WID=`getWID rviz side_view.rviz`
wmctrl -ia $SIDE_VIEW_RVIZ_WID
wmctrl -ir $SIDE_VIEW_RVIZ_WID -e 0,800,0,800,900

DEFAULT_RVIZ_WID=`getWID rviz rviz.rviz`
wmctrl -ir $DEFAULT_RVIZ_WID -b add,hidden
