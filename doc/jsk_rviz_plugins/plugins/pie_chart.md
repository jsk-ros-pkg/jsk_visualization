# PieChart
Plot a pie chart of `std_msgs/Float32` on rviz as HUD overlay.

![](images/pie_chart.png)

To change caption text, please [rename plugin display name](http://docs.ros.org/jade/api/rviz/html/user_guide/#naming-displays) on rviz Displays tab

## Properties
* `Topic`

  `std_msgs::Float32` topic to subscribe to
* `size`

  Size of the plotter window
* `left`

  Left of the plotter window
* `top`

  Top of the plotter window
* `foreground color`
  
  Color to draw line
* `foreground alpha`
  
  Alpha belnding value for foreground
* `foreground alpha2`

  Alpha belnding value for foreground for indicator
* `background color`

  Background color
* `background alpha`

  Alpha belnding value for background
* `text size`

  Text size
* `show caption`

  Show caption
* `max value`

  Max value of pie chart
* `min value`

  Min value of pie chart
* `auto color change`

  Change the color automatically
* `max color`
 
  Max color of pie chart 
  
  Only used if auto color change is set to True
* `med color`

  Med color of pie chart
  
  Only used if auto color change is set to True
* `max color change threshold`

  Change the max color at this threshold
  
  Only used if auto color change is set to True
* `med color change threshold`

  Change the med color at this threshold
  
  Only used if auto color change is set to True
* `clockwise rotate direction`

  Change the rotate direction


## Sample
```
roslaunch jsk_rviz_plugins overlay_sample.launch
```
  or
```
roslaunch jsk_rviz_plugins piechart_sample.launch
```
