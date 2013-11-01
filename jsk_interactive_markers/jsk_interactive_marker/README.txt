# To use robot joint interactive marker
roslaunch jsk_interactive_marker urdf_model_markers.launch models:=/path/to/model.yaml


# for example
roslaunch jsk_interactive_marker urdf_model_markers.launch models:=`rospack find jsk_interactive_marker`/launch/models/urdf_model_markers.yaml

# If you want to use atlas_v3 model, please type the following command
roscd jsk_interactive_marker/urdf
./atlas_v3.sh
roslaunch jsk_interactive_marker urdf_model_markers.launch models:=`rospack find jsk_interactive_marker`/launch/models/atlas-sample.yaml


# To view in rviz, add Interactive Marker and select a topic
  after you run rviz
