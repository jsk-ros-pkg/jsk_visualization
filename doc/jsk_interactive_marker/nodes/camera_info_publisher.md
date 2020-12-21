# camera_info_publisher

`camera_info_publisher` provides camera info topics to an image or pointcloud without camera info.


## Parameters

* `~yaml_filename` (String, default: ``)

   Path to yaml file which has camera info information.

* `~frame_id` (String, default: `camera`)

   Frame id of camera info.

* `~parent_frame_id_` (Bool, default: `base_link`)

   Frame id of interactive marker.

* `~sync_pointcloud` (String, default: `false`)

    Synchronize camera info to pointcloud.
    If both `~sync_pointcloud` and `~sync_image` are not specified, camera info is published at a static rate.

* `~sync_image` (String, default: `false`)

    Synchronize camera info to image.

* `~static_rate` (Double, default: `30.0`)

    Static rate at which camera info is published.
    If both `~sync_pointcloud` and `~sync_image` are not specified, camera info is published at a static rate.

* `~width` (Double, default: `640`)

    With of published camera info. This parameter is enabled when `~yaml_filename` is not speficied. This parameter can be changed by dynamic reconfigure.

* `~height` (Double, default: `480`)

    Height of published camera info. This parameter is enabled when `~yaml_filename` is not speficied. This parameter can be changed by dynamic reconfigure.

* `~f` (Double, default: `525`)

    F of published camera_info. This parameter is enabled when `~yaml_filename` is not speficied. This parameter can be changed by dynamic reconfigure.


## Subscribing Topics

* `~input` (`sensor_msgs/Image` or `sensor_msgs/Pointcloud2`)

   Image or pointcloud whose camera info is published.


## Publishing Topics

* `~camera_info` (`sensor_msgs/CameraInfo`)

   Camera info which has the same timestamp as the input topic.

## Sample

```
roslaunch jsk_interactive_marker sample_camera_info_publisher.launch
```
