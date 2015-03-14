#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import *
from geometry_msgs.msg import Point
rospy.init_node("test_occupancy_grid")

p = rospy.Publisher("/occupancy_grid", SimpleOccupancyGridArray)

r = rospy.Rate(1)

def cells():
    ret = []
    for i in range(0, 100):
        for j in range(0, 100):
            ret.append(Point(x = 0.01 * i, y = 0.01 * j, z = 0))
    return ret


while not rospy.is_shutdown():
    now = rospy.Time.now()
    occupancy_grid = SimpleOccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.header.stamp = now
    occupancy_grid.coefficients = [0, 0, 1, 0]
    occupancy_grid.resolution = 0.01      #1cm resolution
    occupancy_grid.cells = cells()
    occupancy_grid2 = SimpleOccupancyGrid()
    occupancy_grid2.header.frame_id = "map"
    occupancy_grid2.header.stamp = now
    occupancy_grid2.coefficients = [0, 0, 1, 1]
    occupancy_grid2.resolution = 0.01      #1cm resolution
    occupancy_grid2.cells = cells()
    occupancy_grid_array = SimpleOccupancyGridArray()
    occupancy_grid_array.grids = [occupancy_grid, occupancy_grid2]
    occupancy_grid_array.header.stamp = now
    occupancy_grid_array.header.frame_id = "map"
    p.publish(occupancy_grid_array)
    r.sleep()    
