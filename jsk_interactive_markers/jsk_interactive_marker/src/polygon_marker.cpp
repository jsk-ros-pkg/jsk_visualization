/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <interactive_markers/interactive_marker_server.h>
#include <jsk_recognition_msgs/Int32Stamped.h>
#include <jsk_interactive_marker/IndexRequest.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <jsk_recognition_utils/geo/polygon.h>
#include <jsk_topic_tools/log_utils.h>

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
boost::mutex mutex;
ros::Publisher pub, polygon_pub, polygon_arr_pub;
jsk_recognition_msgs::PolygonArray::ConstPtr polygon_msg;

void publishClickedPolygon(jsk_recognition_msgs::Int32Stamped& msg)
{
  pub.publish(msg);
  polygon_pub.publish(polygon_msg->polygons[msg.data]);
  jsk_recognition_msgs::PolygonArray array_msg;
  array_msg.header = polygon_msg->header;
  array_msg.polygons.push_back(polygon_msg->polygons[msg.data]);
  polygon_arr_pub.publish(array_msg);
}

void processFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // control_name is "sec nsec index"
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
    std::string control_name = feedback->control_name;
    ROS_INFO("control_name: %s", control_name.c_str());
    std::list<std::string> splitted_string;
    boost::split(splitted_string, control_name, boost::is_space());
    jsk_recognition_msgs::Int32Stamped index;
    index.header.stamp.sec = boost::lexical_cast<int>(splitted_string.front());
    splitted_string.pop_front();
    index.header.stamp.nsec = boost::lexical_cast<int>(splitted_string.front());
    splitted_string.pop_front();
    index.data = boost::lexical_cast<int>(splitted_string.front());
    publishClickedPolygon(index);
  }
}

// bool indexRequest(jsk_interactive_marker::IndexRequest::Request  &req,
//                   jsk_interactive_marker::IndexRequest::Response &res)
// {
//   publishClickedBox(req.index);
// }

void polygonCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
{
  polygon_msg = msg;
  server->clear();
  // create cube markers
  for (size_t i = 0; i < msg->polygons.size(); i++) {
    geometry_msgs::PolygonStamped polygon = msg->polygons[i];
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = polygon.header.frame_id;
    int_marker.pose.orientation.w = 1;
    {
      std::stringstream ss;
      ss << "polygon" << "_" << i;
      int_marker.name = ss.str();
    }
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    
    {
      std::stringstream ss;
      // encode several informations into control name
      ss << polygon.header.stamp.sec << " " << polygon.header.stamp.nsec << " " << i;
      control.name = ss.str();
    }
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.0;
    jsk_recognition_utils::Polygon::Ptr polygon_obj
      = jsk_recognition_utils::Polygon::fromROSMsgPtr(polygon.polygon);
    std::vector<jsk_recognition_utils::Polygon::Ptr> decomposed_triangles
      = polygon_obj->decomposeToTriangles();
    for (size_t j = 0; j < decomposed_triangles.size(); j++) {
      jsk_recognition_utils::Vertices vs
        = decomposed_triangles[j]->getVertices();
      for (size_t k = 0; k < vs.size(); k++) {
        geometry_msgs::Point p;
        p.x = vs[k][0];
        p.y = vs[k][1];
        p.z = vs[k][2];
        marker.points.push_back(p);
      }
    }
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
  }
  server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_interactive_marker");
  ros::NodeHandle n, pnh("~");
  server.reset(new interactive_markers::InteractiveMarkerServer("polygon_interactive_marker", "", false));
  pub = pnh.advertise<jsk_recognition_msgs::Int32Stamped>("selected_index", 1);
  polygon_pub = pnh.advertise<geometry_msgs::PolygonStamped>("selected_polygon", 1);
  polygon_arr_pub = pnh.advertise<jsk_recognition_msgs::PolygonArray>("selected_polygon_array", 1);
  ros::Subscriber sub = pnh.subscribe("polygon_array", 1, polygonCallback);
  ros::spin();
  server.reset();
  return 0;
}
