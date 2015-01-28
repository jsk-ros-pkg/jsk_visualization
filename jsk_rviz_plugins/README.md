# jsk\_rviz\_plugins

## Introduction
jsk\_rviz\_plugins is a package to provide original rviz plugins.

You can use this rviz plugins just launch rviz.

## rviz\_plugins

### Displays
#### AmbientSound
#### Diagnostics
#### FootStep
#### Normal

##### What Is This

This will show the Normal which is subcribed from topic (sensor_msgs::PointCloud2).
The normal is assumed to have the features x,y,z,normal\_x,normal\_y,normal\_z.

![Normal Plugin](cfg/image/normal_sample.png "Normal Plugins in RViz")

##### Samples
Plug the depth sensor which could be launched by openni.launch and execute below command.

```
roslaunch jsk_rviz_plugins normal_sample.launch
```

#### [TwistStamped](http://youtu.be/Q-I5Vx_4VHk)
![](images/twist_stamped.png)

[Movie](http://youtu.be/Q-I5Vx_4VHk)

Visualize `geometry_msgs/TwistStamped` by arrows. Linear velocity is represented by one arrow
and angular velocity is represented by 3 arrows for each axis.

##### Properties
* linear scale (default: `1.0`)
* angular scale (default: `1.0`)

  Scale factor of size of arrows

* linear color (default: `RGB(0, 255, 0)`)
* angular color (default: `RGB(255, 0, 0)`)

   Color of arrows

------

#### OverlayText
#### PieChart
#### Plotter2D
##### What Are These


These will show text or graph on the rviz main view.

![Plotter2D PieChart OverlayText Plugin](cfg/image/overlay_sample.png "Overlay Plugins in RViz")

##### Samples

Just run below commands

```
roslaunch jsk_rviz_plugins overlay_sample.launch
```

---

#### PolygonArray

#### Pictogram
![Pictogram](images/pictogram.png)

[movie](https://www.youtube.com/watch?v=AJe1uQupsUE)

Pictogram is a rviz plugin to visualize icons.
Pictogram plugin uses [Entypo](http://entypo.com/) and [FontAwesome](http://fortawesome.github.io/Font-Awesome/).

You need to use `jsk_rviz_plugins/Pictogram` message to use it.

You can find mapping with `character` and icons at [here](http://fortawesome.github.io/Font-Awesome/icons/) and [here](http://entypo.com/characters/).

### Panels
#### CancelAction
#### PublishTopic
#### SelectPointCloudPublishAction
