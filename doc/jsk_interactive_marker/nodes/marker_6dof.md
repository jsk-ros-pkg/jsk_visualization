# marker_6dof

![](../images/marker_6dof.png)

`marker_6dof` provides interactive marker to control a marker of primitive shape or mesh shape.


## Parameters

* `~object_type` (String, default: `sphere`)

   Type of object shape. cube, sphere, line, and mesh are available.
* `~frame_id` (String, default: `/map`)

   Frame id of marker.

* `~publish_tf` (Bool, default: `False`)

   Tf of marker pose is published if true.
* `~tf_frame` (String, default: `object`)

    frame id of published tf. This value is used only when `~publish_tf` is true.
* `~publish_pose_periodically` (Bool, default: `False`)

   Pose of marker is published periodically if true.
   Pose topic is published only when marker is moved via Rviz if false.
* `~object_x` (Double, default: `1.0`)

   X scale of object.
* `~object_y` (Double, default: `1.0`)

   Y scale of object.
* `~object_z` (Double, default: `1.0`)

   Z scale of object.
* `~object_r` (Double, default: `1.0`)

   Red color of object.
* `~object_g` (Double, default: `1.0`)

   Green color of object.
* `~object_b` (Double, default: `1.0`)

   Blue color of object.
* `~object_a` (Double, default: `1.0`)

   Alpha value of object color.

* `~initial_x` (Double, default: `0.0`)

   Initial x position of marker.
* `~initial_y` (Double, default: `0.0`)

   Initial y position of marker.
* `~initial_z` (Double, default: `0.0`)

   Initial z position of marker.


## Topics
* /feedback (`visualization_msgs/InteractiveMarkerFeedback`)
* /move_marker (`geometry_msgs/PoseStamped`)
* /update (`visualization_msgs/InteractiveMarkerUpdate`)
* /update_full (`visualization_msgs/InteractiveMarkerInit`)

You can control markers through topics above.

* /pose (`geometry_msgs/PoseStamped`)

Pose of marker.
You can select publishing polcy via `~publish_pose_periodically`.

* /tf (`tf2_msgs/TFMessage`)

Tf of marker pose. Available only when `~publish_tf` is true.


## Sample

```
roslaunch jsk_interactive_marker marker_6dof_sample.launch
```
