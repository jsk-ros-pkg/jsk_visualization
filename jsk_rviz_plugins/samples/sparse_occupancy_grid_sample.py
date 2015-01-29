#!/usr/bin/env python

try:
  from jsk_recognition_msgs.msg import *
except:
  import roslib; roslib.load_manifest("jsk_rviz_plugins")
  from jsk_recognition_msgs.msg import *
import rospy
import random

rospy.init_node("test_occupancy_grid")

p = rospy.Publisher("/sparse_occupancy_grid", SparseOccupancyGridArray)

r = rospy.Rate(1)

def buildGrid(indices, position, quaternion):
  grid = SparseOccupancyGrid()
  grid.resolution = 0.1
  grid.origin_pose.position.x = position[0]
  grid.origin_pose.position.y = position[1]
  grid.origin_pose.position.z = position[2]
  grid.origin_pose.orientation.x = quaternion[0]
  grid.origin_pose.orientation.y = quaternion[1]
  grid.origin_pose.orientation.z = quaternion[2]
  grid.origin_pose.orientation.w = quaternion[3]
  grid.header.stamp = rospy.Time.now()
  grid.header.frame_id = "map"
  for column_indices in indices:
    column = SparseOccupancyGridColumn()
    column.column_index = column_indices[0][0]
    for (ci, ri) in column_indices:
      cell = SparseOccupancyGridCell()
      cell.row_index = ri
      cell.value = random.random()
      column.cells.append(cell)
    grid.columns.append(column)
  return grid

while not rospy.is_shutdown():
  # buildup occupancy grid
  grid1_indices = [[[-1] + [i] for i in range(20)], 
                   [[0] + [i] for i in range(20)], 
                   [[-2] + [i] for i in range(20)], 
                   [[-3] + [i] for i in range(20)], 
                   [[-4] + [i] for i in range(20)]]
  grid2_indices = [[[-1] + [i] for i in range(20)], 
                   [[0] + [i] for i in range(20)], 
                   [[1] + [i] for i in range(20)], 
                   [[2] + [i] for i in range(20)], 
                   [[3] + [i] for i in range(20)], 
                   [[4] + [i] for i in range(20)]]
  grid1 = buildGrid(grid1_indices, [0, 0, 0], [0.087156, 0, 0, 0.996195])
  grid2 = buildGrid(grid2_indices, [0, 0, 1], [-0.087156, 0, 0, 0.996195])
  grid_array = SparseOccupancyGridArray()
  grid_array.header.stamp = rospy.Time.now()
  grid_array.header.frame_id = "map"
  grid_array.grids = [grid1, grid2]
  p.publish(grid_array)
  r.sleep()
