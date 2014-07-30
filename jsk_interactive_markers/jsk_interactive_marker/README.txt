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


# You can select "Robot Mode" or "Object Mode".
# In yaml file, "robot: true"  means "Robot Mode". "robot: false"
# means "Object Mode".

# In Robot mode, you can move each joint in rviz.
# When you move marker, "nodeName/robotName/joint_states" is
# published.
# If you publish "nodeName/robotName/reset_joint_states/" , robot
# markers are set according to this joint angle.

# When you press "Move" or "Reset Marker Pose", "nodeName/marker_menu"
# is published.
# Please write an apropriate code in order to move a robot and reset
# marker to a pose of robot.

# The sample code written in Euslisp is in jsk_interactive.
# To run the sample, type the following command.
rosrun roseus roseus `rospack find jsk_interactive`/euslisp/joint_marker_control/sample-pr2-joint.l
# or
rosrun roseus roseus `rospack find jsk_interactive`/euslisp/joint_marker_control/altas-joint.l
