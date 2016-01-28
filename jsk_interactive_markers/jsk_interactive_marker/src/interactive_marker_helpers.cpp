/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// author: Adam Leeper

//#include <interactive_marker_helpers/interactive_marker_helpers.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <tf/tf.h>
// #include <object_manipulator/tools/msg_helpers.h>

namespace im_helpers {

geometry_msgs::Pose createPoseMsg(const tf::Transform &transform)
{
  geometry_msgs::Pose pose;
  tf::Vector3 pos = transform.getOrigin();
  tf::Quaternion rot = transform.getRotation();
  pose.position.x = pos[0];
  pose.position.y = pos[1];
  pose.position.z = pos[2];
  pose.orientation.x = rot.getX();
  pose.orientation.y = rot.getY();
  pose.orientation.z = rot.getZ();
  pose.orientation.w = rot.getW();
  return pose;
}
  
visualization_msgs::InteractiveMarker makeEmptyMarker( const char *frame_id )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;

  return int_marker;
}

visualization_msgs::Marker makeBox( float scale )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker makeSphere( float scale )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

void add3Dof2DControl( visualization_msgs::InteractiveMarker &msg, bool fixed)
{
  visualization_msgs::InteractiveMarkerControl control;

  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  // msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  // msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  // msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

}

void add6DofControl( visualization_msgs::InteractiveMarker &msg, bool fixed)
{
  visualization_msgs::InteractiveMarkerControl control;

  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);
}

  void addVisible6DofControl( visualization_msgs::InteractiveMarker &msg, bool fixed, bool visible)
  {
  visualization_msgs::InteractiveMarkerControl control;
  
  if(visible)
    control.always_visible = true;

  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);
}


visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& makeSphereControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeSphere(msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::MenuEntry makeMenuEntry(const char *title)
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = title;
  return m;
}

visualization_msgs::MenuEntry makeMenuEntry(const char *title, const char *command, int type  )
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = command;
  m.command_type = type;
  return m;
}

visualization_msgs::InteractiveMarker makePostureMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float scale, bool fixed, bool view_facing )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  return int_marker;
}

visualization_msgs::InteractiveMarker makeHeadGoalMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                          float scale)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.orientation.w = 1;
  control.markers.push_back( makeSphere(scale*0.7) );
  int_marker.controls.push_back(control);
  control.markers.clear();

  add6DofControl(int_marker);

  return int_marker;
}

visualization_msgs::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color, bool use_color )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.pose = stamped.pose;
  int_marker.name = name;
  int_marker.scale = scale;

  visualization_msgs::Marker mesh;
  if (use_color) mesh.color = color;
  mesh.mesh_resource = mesh_resource;
  mesh.mesh_use_embedded_materials = !use_color;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = scale;
  mesh.scale.y = scale;
  mesh.scale.z = scale;

  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back( mesh );
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back( control );

  return int_marker;
}

visualization_msgs::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::PoseStamped &stamped, float scale)
{
  std_msgs::ColorRGBA color;
  return makeMeshMarker( name, mesh_resource, stamped, scale, color, false);
}

visualization_msgs::InteractiveMarker makeMeshMarker( const std::string &name, const std::string &mesh_resource,
                                                      const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color)
{
  return makeMeshMarker( name, mesh_resource, stamped, scale, color, true);
}

visualization_msgs::InteractiveMarker makeButtonBox( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed, bool view_facing )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeBoxControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeButtonSphere( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                        float scale, bool fixed, bool view_facing)
{
  std_msgs::ColorRGBA color;
  return makeButtonSphere(name, stamped, scale, fixed, view_facing, color);
}

visualization_msgs::InteractiveMarker makeButtonSphere( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                        float scale, bool fixed, bool view_facing,
                                                        std_msgs::ColorRGBA color)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeSphereControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.markers.back().color = color;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeListControl( const char *name, const geometry_msgs::PoseStamped &stamped, int num, int total, float scale)
{

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;

//  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  std::stringstream ss;
  ss << "pose(" << num << "/" << total << ")";
  interactive_markers::makeViewFacingButton(int_marker, control, ss.str());

//  control.orientation.w = 1;
//  control.orientation.x = 0;
//  control.orientation.y = -1;
//  control.orientation.z = 0;

//  interactive_markers::makeArrow( int_marker, control, 0.3*scale );
//  control.markers.back().color.r = 1.0;
//  control.markers.back().color.g = 0.0;
//  control.markers.back().color.b = 0.0;
//  control.name = "right";
//  int_marker.controls.push_back(control);
//
//  control.markers.clear();
//  interactive_markers::makeArrow( int_marker, control, -0.3*scale );
//  control.markers.back().color.r = 0.0;
//  control.markers.back().color.g = 1.0;
//  control.markers.back().color.b = 0.0;
//  control.name = "left";
  int_marker.controls.push_back(control);

  return int_marker;


//  visualization_msgs::InteractiveMarker int_marker;
//  int_marker.header =  stamped.header;
//  int_marker.name = name;
//  int_marker.scale = scale;
//  int_marker.pose = stamped.pose;
//  //int_marker.description = "This is the marker.";
//
//  visualization_msgs::InteractiveMarkerControl &control = makeSphereControl(int_marker);
//  //control.description = "This is the control";
//  control.always_visible = false;
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
//
//  return int_marker;
}

visualization_msgs::InteractiveMarker make6DofMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed, bool view_facing )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  if ( view_facing )
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back( makeSphere(scale*0.5) );
    int_marker.controls.push_back(control);
  }
  else
  {
    add6DofControl(int_marker, fixed);
  }

  return int_marker;
}

visualization_msgs::InteractiveMarker makePlanarMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = fixed ? (uint8_t)visualization_msgs::InteractiveMarkerControl::FIXED : (uint8_t)visualization_msgs::InteractiveMarkerControl::INHERIT;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  interactive_markers::makeArrow( int_marker, control, 0 );
  control.markers.back().pose.orientation.w = 0;
  control.markers.back().pose.orientation.x = 0;
  control.markers.back().pose.orientation.y = 1;
  control.markers.back().pose.orientation.z = 0;
  control.markers.back().color.r = 0.0;
  control.markers.back().color.g = 1.0;
  control.markers.back().color.b = 0.0;
  int_marker.controls.push_back(control);

  return int_marker;
}

visualization_msgs::InteractiveMarker makeElevatorMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;

  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = -1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  interactive_markers::makeArrow( int_marker, control, 0.3 );
  control.markers.back().color.r = 0.0;
  control.markers.back().color.g = 1.0;
  control.markers.back().color.b = 0.0;
  control.name = "up";
  int_marker.controls.push_back(control);

  control.markers.clear();
  interactive_markers::makeArrow( int_marker, control, -0.3 );
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 0.0;
  control.markers.back().color.b = 0.0;
  control.name = "down";
  int_marker.controls.push_back(control);

  return int_marker;
}

visualization_msgs::InteractiveMarker makeProjectorMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.04;
  marker.color.r = 1.0;
  marker.color.a = 0.8;
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  return int_marker;
}


visualization_msgs::InteractiveMarker makeBaseMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.orientation.y = -1;
  int_marker.controls.push_back(control);


  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.orientation.w = 1;
  control.orientation.y = 0;

  control.markers.clear();
  interactive_markers::makeArrow( int_marker, control, 0.9 );
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 0.0;
  control.markers.back().color.b = 0.0;
  control.name = "forward";
  int_marker.controls.push_back(control);

  control.markers.clear();
  interactive_markers::makeArrow( int_marker, control, -0.9 );
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 0.0;
  control.markers.back().color.b = 0.0;
  control.name = "back";
  int_marker.controls.push_back(control);

  control.orientation.z = 1;
  control.markers.clear();
  interactive_markers::makeArrow( int_marker, control, 0.9 );
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 0.0;
  control.markers.back().color.b = 0.0;
  control.name = "left";
  int_marker.controls.push_back(control);

  control.markers.clear();
  interactive_markers::makeArrow( int_marker, control, -0.9 );
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 0.0;
  control.markers.back().color.b = 0.0;
  control.name = "right";
  int_marker.controls.push_back(control);

  control.markers.clear();
  tf::quaternionTFToMsg( tf::Quaternion(tf::Vector3(0,0,1), 135*M_PI/180.0), control.orientation);
  interactive_markers::makeArrow( int_marker, control, 1.0 );
  control.markers.back().pose.position.x = 0.7;
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 1.0;
  control.markers.back().color.b = 0.0;
  control.name = "rotate left";
  int_marker.controls.push_back(control);

  control.markers.clear();
  tf::quaternionTFToMsg( tf::Quaternion(tf::Vector3(0,0,1), -135*M_PI/180.0), control.orientation);
  interactive_markers::makeArrow( int_marker, control, 1.0 );
  control.markers.back().pose.position.x = 0.7;
  control.markers.back().color.r = 1.0;
  control.markers.back().color.g = 1.0;
  control.markers.back().color.b = 0.0;
  control.name = "rotate right";
  int_marker.controls.push_back(control);
  return int_marker;
}

visualization_msgs::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing)
{
  std_msgs::ColorRGBA color;
  return makeGripperMarker( name, stamped, scale, angle, view_facing, color, false);
}

visualization_msgs::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing, std_msgs::ColorRGBA color )
{
  return makeGripperMarker( name, stamped, scale, angle, view_facing, color, true);
}


visualization_msgs::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing, std_msgs::ColorRGBA color, bool use_color )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = 1.0; //scale;
  int_marker.pose = stamped.pose;

//  add6DofControl(int_marker, false);

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh;
  mesh.mesh_use_embedded_materials = !use_color;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = scale;
  mesh.scale.y = scale;
  mesh.scale.z = scale;
  mesh.color = color;

  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;

  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  control.markers.push_back( mesh );

  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  control.markers.push_back( mesh );

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );

  return int_marker;
}


visualization_msgs::InteractiveMarker makeGraspMarker( const char * name, const geometry_msgs::PoseStamped &stamped, float scale, PoseState pose_state)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.header.stamp = ros::Time(0);
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.orientation.w = 1;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  //marker.header = int_marker.header;
  //marker.pose = int_marker.pose;
  marker.scale.x = 0.025*scale;
  marker.scale.y = 0.0025*scale;
  marker.scale.z = 0.0025*scale;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  control.markers.push_back(marker);
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.015*scale;
  marker.scale.y = 0.04*scale;
  marker.scale.z = 0.015*scale;
  switch (pose_state)
  {
  case UNTESTED:
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.r = 0.5;
    break;
  case VALID:
    marker.color.g = 1.0;
    marker.color.r = 0.0;
    break;
  case INVALID:
    marker.color.g = 0.0;
    marker.color.r = 1.0;
    break;
  }
  control.markers.push_back(marker);
//  marker.points[0] = object_manipulator::msg::createPointMsg(0,0,0);
//  marker.points[1] = object_manipulator::msg::createPointMsg(0,0.03,0);
//  marker.color.r = 0;
//  marker.color.g = 1.0;
//  control.markers.push_back(marker);

  int_marker.controls.push_back(control);

  return int_marker;
}


visualization_msgs::InteractiveMarker makePosedMultiMeshMarker( const char * name, const geometry_msgs::PoseStamped &stamped,
                                                                const std::vector< geometry_msgs::PoseStamped> &mesh_poses,
                                                                const std::vector<std::string> &mesh_paths, const float &scale,
                                                                const bool button_only)
{
    //ROS_ERROR("Start create robot marker...");
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header =  stamped.header;
    int_marker.name = name;
    int_marker.scale = scale;
    int_marker.pose = stamped.pose;

    bool fixed = false;
    visualization_msgs::InteractiveMarkerControl control;
    if( !button_only )
    {
        control.orientation_mode = fixed ? (uint8_t)visualization_msgs::InteractiveMarkerControl::FIXED : (uint8_t)visualization_msgs::InteractiveMarkerControl::INHERIT;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        int_marker.controls.push_back(control);
    }

    control.markers.clear();
    if (button_only) {
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    }
    else {
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    }
    control.always_visible = true;
    for(size_t i = 0; i < mesh_poses.size(); i++)
    {
        visualization_msgs::Marker mesh;
        mesh.color = std_msgs::ColorRGBA();
        mesh.mesh_resource = mesh_paths[i];
        mesh.mesh_use_embedded_materials = true;
        mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        mesh.scale.x = mesh.scale.y = mesh.scale.z = scale;
        mesh.pose = mesh_poses[i].pose;
        control.markers.push_back( mesh );
    }
    int_marker.controls.push_back( control );

    return int_marker;

}

visualization_msgs::InteractiveMarker makeFollowerMultiMeshMarker( const char * name, const geometry_msgs::PoseStamped &stamped,
                                                                   const std::vector<std::string> &mesh_frames,
                                                                   const std::vector<std::string> &mesh_paths,
                                                                   const float &scale)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header =  stamped.header;
    int_marker.name = name;
    int_marker.scale = scale;
    int_marker.pose = stamped.pose;

    if(mesh_frames.size() != mesh_paths.size())
    {
      ROS_ERROR("The number of mesh frames and mesh paths is not equal!");
      return int_marker;
    }

    visualization_msgs::InteractiveMarkerControl control;

    control.markers.clear();
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;
    for(size_t i = 0; i < mesh_frames.size(); i++)
    {
        visualization_msgs::Marker mesh;
        mesh.color = std_msgs::ColorRGBA();
        mesh.mesh_resource = mesh_paths[i];
        mesh.mesh_use_embedded_materials = true;
        mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        mesh.scale.x = mesh.scale.y = mesh.scale.z = scale;
        mesh.pose.orientation.w = 1;
        mesh.header.frame_id = mesh_frames[i];
        control.markers.push_back( mesh );
        mesh.frame_locked = true;
    }
    int_marker.controls.push_back( control );

    return int_marker;
}

} // namespace im_helpers
