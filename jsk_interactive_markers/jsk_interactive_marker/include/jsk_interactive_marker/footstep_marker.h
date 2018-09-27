// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_interactive_marker/FootstepMarkerConfig.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <jsk_footstep_msgs/PlanFootstepsAction.h>
#include <jsk_footstep_msgs/ExecFootstepsAction.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <jsk_recognition_msgs/SimpleOccupancyGridArray.h>
#include <dynamic_reconfigure/server.h>

class FootstepMarker {
public:
  typedef jsk_interactive_marker::FootstepMarkerConfig Config;
  FootstepMarker();
  virtual ~FootstepMarker();
  void updateInitialFootstep();
  typedef actionlib::SimpleActionClient<jsk_footstep_msgs::PlanFootstepsAction>
  PlanningActionClient;
  typedef actionlib::SimpleActionClient<jsk_footstep_msgs::ExecFootstepsAction>
  ExecuteActionClient;
  typedef jsk_footstep_msgs::PlanFootstepsResult PlanResult;
protected:
  void initializeInteractiveMarker();
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void menuFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void menuCommandCB(const std_msgs::UInt8::ConstPtr& msg);
  void executeCB(const std_msgs::Empty::ConstPtr& msg);
  void resumeCB(const std_msgs::Empty::ConstPtr& msg);
  void planDoneCB(const actionlib::SimpleClientGoalState &state, 
                  const PlanResult::ConstPtr &result);
  void processMenuFeedback(uint8_t id);
  geometry_msgs::Polygon computePolygon(uint8_t leg);
  void snapLegs();
  geometry_msgs::Pose computeLegTransformation(uint8_t leg);
  geometry_msgs::Pose getFootstepPose(bool leftp);
  void changePlannerHeuristic(const std::string& heuristic);
  void callEstimateOcclusion();
  void cancelWalk();
  void planIfPossible();
  void resetLegPoses();
  void lookGround();
  void configCallback(Config& config, uint32_t level);
  bool forceToReplan(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res);
  boost::mutex plane_mutex_;
  boost::mutex plan_run_mutex_;
  std::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  // projection to the planes
  bool projectMarkerToPlane();
  
  jsk_recognition_msgs::SimpleOccupancyGridArray::ConstPtr latest_grids_;
  // read a geometry_msgs/pose from the parameter specified.
  // the format of the parameter is [x, y, z, xx, yy, zz, ww].
  // where x, y and z means position and xx, yy, zz and ww means
  // orientation.
  void readPoseParam(ros::NodeHandle& pnh, const std::string param,
                     tf::Transform& offset);

  // execute footstep
  // sending action goal to footstep controller
  void executeFootstep();
  void resumeFootstep();

  void projectionCallback(const geometry_msgs::PoseStamped& pose);

  visualization_msgs::Marker makeFootstepMarker(geometry_msgs::Pose pose);
  
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  double foot_size_x_;
  double foot_size_y_;
  double foot_size_z_;
  double footstep_margin_;
  std::string marker_frame_id_;
  geometry_msgs::PoseStamped marker_pose_;
  ros::Subscriber move_marker_sub_;
  ros::Subscriber menu_command_sub_;
  ros::Subscriber exec_sub_;
  ros::Subscriber resume_sub_;
  ros::Subscriber projection_sub_;
  ros::Publisher project_footprint_pub_;
  ros::Publisher snapped_pose_pub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher footstep_pub_;
  ros::ServiceClient snapit_client_;
  ros::ServiceClient estimate_occlusion_client_;
  ros::ServiceServer plan_if_possible_srv_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  PlanningActionClient ac_;
  ExecuteActionClient ac_exec_;
  bool use_projection_service_;
  bool use_projection_topic_;
  bool show_6dof_control_;
  bool use_footstep_planner_;
  bool use_footstep_controller_;
  bool plan_run_;
  bool use_plane_snap_;
  bool wait_snapit_server_;
  bool use_initial_footstep_tf_;
  bool use_initial_reference_;
  bool always_planning_;
  bool lleg_first_;
  bool use_2d_;
  std::string initial_reference_frame_;
  geometry_msgs::Pose lleg_pose_;
  geometry_msgs::Pose rleg_pose_;
  geometry_msgs::Pose lleg_initial_pose_;
  geometry_msgs::Pose rleg_initial_pose_;
  tf::Transform lleg_offset_;
  tf::Transform rleg_offset_;
  std::string lfoot_frame_id_;
  std::string rfoot_frame_id_;

  // footstep plannner result
  PlanResult::ConstPtr plan_result_;
};
