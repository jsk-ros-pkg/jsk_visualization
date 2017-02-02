# transformable_markers_client.py

![](images/transformable_markers_client.gif)

`transformable_markers_client.py` has features below:

- Insert markers to `transformable_server_sample`
- Auto save of user interaction of the markers
- Publish topics of reusable msgs after conversion from the markers (ex. BOX -> BoundingBox)


## Parameters

* `~config_file` (String, requried)

  Config file to insert markers, and auto save the interaction.
  The format is like below:

```yaml
boxes:
- name: box_a                        # (requried)
  frame_id: map                      # (optional, default: /map)
  dimensions: [0.09, 0.13, 0.15]     # (optional, default: [1, 1, 1])
  position: [1, 0, 0]                # (optional, default: [0, 0, 0, 1])
  orientation: [0.0, 0.0, 0.0, 1.0]  # (optional, default: [0, 0, 1])
...
```

* `~config_auto_save` (Bool, default: True)

  Enable the feature to save the config automatically.


## Required ROS name

* `~server`

  Node name of `transformable_server_sample` server.


## Publishing Topics

* `~output/boxes` (`jsk_recognition_msgs/BoundingBoxArray`)

  Converted boxes from marker: BOX -> BoundingBox.


## Sample

```bash
roslaunch jsk_interactive_marker sample_transformable_markers_client.launch
```
