#ifndef INTERACTIVE_POINT_CLOUD
#define INTERACTIVE_POINT_CLOUD

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/BoundingBoxArray.h>
#include <jsk_pcl_ros/BoundingBoxMovement.h>
#include <jsk_pcl_ros/Int32Stamped.h>
#include <geometry_msgs/PoseArray.h>

#include <actionlib/client/simple_action_client.h>
//#include <point_cloud_server/StoreCloudAction.h>
#include <pcl/search/kdtree.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//#include <object_manipulator/tools/mechanism_interface.h>
#include "jsk_interactive_marker/InteractivePointCloudConfig.h"
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointXYZRGB PointT;


class InteractivePointCloud
{
public:

  InteractivePointCloud(
                std::string marker_name,
                std::string topic_name, std::string server_name);

  ~InteractivePointCloud();

  //! Clear the cloud stored in this object
  void hide(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void handlePoseCallback(const geometry_msgs::PoseStampedConstPtr &ps);
  void pointCloudAndBoundingBoxCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const jsk_pcl_ros::BoundingBoxArrayConstPtr &box);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  //  void handlePoseAndBoundingBoxCallback(const geometry_msgs::PoseStampedConstPtr &ps, const jsk_pcl_ros::BoundingBoxArrayConstPtr &box);
  void setHandlePoseCallback(const geometry_msgs::PoseStampedConstPtr &ps);
  void handlePoseAndBoundingBoxCallback(const jsk_pcl_ros::Int32StampedConstPtr &index, const geometry_msgs::PoseArrayConstPtr &pa, const jsk_pcl_ros::BoundingBoxArrayConstPtr &box);
private:

  typedef jsk_interactive_marker::InteractivePointCloudConfig Config;
  boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
  boost::mutex mutex_;
  virtual void configCallback (Config &config, uint32_t level);

  typedef interactive_markers::MenuHandler MenuHandler;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, jsk_pcl_ros::BoundingBoxArray> SyncPolicy;
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, jsk_pcl_ros::BoundingBoxArray> SyncHandlePose;

  typedef message_filters::sync_policies::ExactTime<jsk_pcl_ros::Int32Stamped, geometry_msgs::PoseArray, jsk_pcl_ros::BoundingBoxArray> SyncHandlePose;
  

  void makeMenu();

  void pickup( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud, float size);
  void makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud, const jsk_pcl_ros::BoundingBoxArrayConstPtr box, float size);

  void menuPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void move( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void leftClickPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishHandPose( geometry_msgs::PoseStamped box_pose);
  

  std::string marker_name_, topic_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_marker_pose_, pub_click_point_, pub_left_click_, pub_handle_pose_, pub_handle_pose_array_, pub_box_movement_;
  ros::Subscriber sub_handle_pose_;
  //ros::Subscriber sub_point_cloud_, sub_bounding_box_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_;
  message_filters::Subscriber<jsk_pcl_ros::BoundingBoxArray> sub_bounding_box_;
  //message_filters::Subscriber<geometry_msgs::PoseStamped> sub_handle_pose_;
  message_filters::Subscriber<jsk_pcl_ros::Int32Stamped> sub_selected_index_;
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_handle_array_;
  tf::TransformListener tfl_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
  boost::shared_ptr<message_filters::Synchronizer<SyncHandlePose> > sync_handle_;

  interactive_markers::InteractiveMarkerServer marker_server_;
  interactive_markers::MenuHandler menu_handler_;

  sensor_msgs::PointCloud2 msg_cloud_;
  double point_size_;
  bool use_bounding_box_;

  geometry_msgs::PoseStamped marker_pose_;
  std::string input_pointcloud_, input_bounding_box_;

  sensor_msgs::PointCloud2 current_croud_;
  jsk_pcl_ros::BoundingBoxArray current_box_;
  geometry_msgs::PoseStamped handle_pose_;
  tf::Transform handle_tf_;
  bool exist_handle_tf_;

  jsk_pcl_ros::BoundingBoxMovement box_movement_;
};

#endif
