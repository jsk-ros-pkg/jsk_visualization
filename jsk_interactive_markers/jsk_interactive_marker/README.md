# Usage
## urdf_model_markers.launch
This launch file generates interactive markers from urdf models.
```
$ roslaunch jsk_interactive_marker urdf_model_markers.launch models:=/path/to/model.yaml
```

### example
```
$ roslaunch jsk_interactive_marker urdf_model_markers.launch models:=`rospack find jsk_interactive_marker`/launch/models/urdf_model_markers.yaml
```

## room2yaml.l
This script generates yaml files for urdf_model_marker.
```
$ roscd jsk_interactive_marker/euslisp
$ rosrun roseus roseus room2yaml.l
```

## robot_actions_sample.launch
This is a sample launch file to test urdf_model_marker and robot-actions.l.
Before launching this, You need to execute room2yaml.l once.
```
$ roslaunch jsk_interactive_marker robot_actions_sample.launch
```
![images/robot_actions_sample_launch.png](images/robot_actions_sample_launch.png)
