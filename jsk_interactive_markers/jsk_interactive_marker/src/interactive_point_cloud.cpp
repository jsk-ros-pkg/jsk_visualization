#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
//#include <point_cloud_server/StoreCloudAction.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>

//#include <pr2_object_manipulation_msgs/CameraFocus.h>

#include <jsk_interactive_marker/interactive_point_cloud.h>

using namespace visualization_msgs;
using namespace interactive_markers;


InteractivePointCloud::InteractivePointCloud(std::string marker_name,
			   std::string topic_name, std::string server_name,
			   std::string cloud_frame):
  marker_name_(marker_name)
  , nh_()
  , pnh_("~")
  , tfl_(nh_)
  , marker_server_(topic_name, server_name, false)
  , cloud_pts_   ( new pcl::PointCloud  <PointT> ()       )
  , cloud_normals_  ( new pcl::PointCloud  <pcl::Normal> ()  )
  , tree_(new pcl::search::KdTree<PointT>())
  //  , cloud_server_client_("point_cloud_server_action", true)
  , cloud_frame_(cloud_frame)
{

  pnh_.param<double>("voxel_size", voxel_size_, 0.010);
  pnh_.param<std::string>("head_pointing_frame", head_pointing_frame_, "/default_head_pointing_frame");

  //  pub_right_click_ = pnh_.advertise<geometry_msgs::PoseStamped>("/right_click_point", 1);
  pub_click_point_ = pnh_.advertise<geometry_msgs::PointStamped>("/right_click_point", 1);
  pub_left_click_ = pnh_.advertise<geometry_msgs::PoseStamped>("/left_click_point", 1);
  pub_refresh_flag_ = pnh_.advertise<std_msgs::String>("refresh_flag", 1);
  //  pub_focus_ = pnh_.advertise<pr2_object_manipulation_msgs::CameraFocus>("camera_focus", 1);

  //sub_point_cloud_ = pnh_.subscribe("/head_mount_kinect/depth_registered/points", 1, 
  sub_point_cloud_ = pnh_.subscribe("/selected_pointcloud", 1, 
				    &InteractivePointCloud::pointCloudCallback, this);

  makeMenu();
}

InteractivePointCloud::~InteractivePointCloud(){};


void InteractivePointCloud::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud){
  ROS_INFO("%lf",voxel_size_);
  ROS_INFO("aaaaa");
  makeMarker(cloud, voxel_size_ * 1.8);
  //updateCloud(*cloud, "aa");
}

void InteractivePointCloud::makeMenu()
{
  // create menu
  menu_handler_.insert( "Broadcast click position",  boost::bind( &InteractivePointCloud::menuPoint, this, _1) );

  menu_handler_.insert( "Clear",    boost::bind( &InteractivePointCloud::clear, this) );
}

void InteractivePointCloud::leftClickPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(!feedback->mouse_point_valid)
  {
    ROS_WARN("Clicked point had an invalid position. Not looking there!");
    return;
  }

  ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                         << feedback->header.frame_id << " at point\n" << feedback->mouse_point );
  
  geometry_msgs::PointStamped click_point;
  click_point.point = feedback->mouse_point;
  click_point.header = feedback->header;
  click_point.header.stamp = ros::Time(0);

}


void InteractivePointCloud::menuPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(feedback->mouse_point_valid)
  {
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                           << feedback->header.frame_id << " at point\n" << feedback->mouse_point );

    geometry_msgs::PointStamped click_point;
    click_point.point = feedback->mouse_point;
    click_point.header = feedback->header;
    click_point.header.stamp = ros::Time(0);
    geometry_msgs::PointStamped head_point;
    head_point.header.frame_id = "/head_tilt_link";
    try{
      tfl_.transformPoint(cloud_frame_, click_point, click_point );
    }
    catch(...)
    {
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                cloud_frame_.c_str(), click_point.header.frame_id.c_str());
      return;
    }
    try{
      tfl_.transformPoint(cloud_frame_, head_point, head_point);
    }
    catch(...)
    {
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                cloud_frame_.c_str(), head_point.header.frame_id.c_str());
      return;
    }
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                           << click_point.header.frame_id << " at point\n" << click_point.point );
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Head point in frame "
                           << head_point.header.frame_id << " at point\n" << head_point.point );

    PointT position; //(point.point.x, point.point.y, point.point.z);
    position.x = click_point.point.x;
    position.y = click_point.point.y;
    position.z = click_point.point.z;
    std::vector< int >   	k_indices(1, 0);
    std::vector< float >  k_sqr_distances(1, 0);
    int N = tree_->nearestKSearch ( position, 1, k_indices, k_sqr_distances);
    if(!N)
    {
      ROS_ERROR("Found no point near clicked point... Serious error... Not broadcasting.");
      return;
    }
    int index = k_indices[0];

    geometry_msgs::PoseStamped ps;
    
    //PointT pt = cloud_pts_->points[index];
    //pcl::Normal norm = cloud_normals_->points[index];

    tf::Vector3 normal = tf::Vector3(cloud_normals_->points[index].normal_x,
                                     cloud_normals_->points[index].normal_y,
                                     cloud_normals_->points[index].normal_z);

    tf::Vector3 point = tf::Vector3(cloud_pts_->points[index].x,
                                    cloud_pts_->points[index].y,
                                    cloud_pts_->points[index].z);

    tf::Vector3 head = tf::Vector3(head_point.point.x, head_point.point.y, head_point.point.z);
    tf::Vector3 point_to_head = (head - point).normalized();
    // We actually want the normal that points away from the head
    if( point_to_head.dot(normal) < 0) normal *= -1;

    ps.pose.position.x = point.x();
    ps.pose.position.y = point.y();
    ps.pose.position.z = point.z();
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Nearest point at:\n" << ps.pose.position );

    tf::Vector3 Z = normal.normalized();
    tf::Vector3 Y = normal.cross(tf::Vector3(0,0,1)).normalized();
    tf::Vector3 X = Y.cross(normal).normalized();
    tf::Matrix3x3 mat(X.x(), Y.x(), Z.x(),
                    X.y(), Y.y(), Z.y(),
                    X.z(), Y.z(), Z.z());
    tf::Quaternion q;
    mat.getRotation(q);
    tf::quaternionTFToMsg(q, ps.pose.orientation);
    ps.header = click_point.header;
    pub_right_click_.publish(ps);
  }
  else
  {
    ROS_WARN("Clicked point had an invalid position. Not broadcasting.");
  }
}


void InteractivePointCloud::get( sensor_msgs::PointCloud2 &cloud)
{
  cloud = msg_cloud_;
}

sensor_msgs::PointCloud2 InteractivePointCloud::get()
{
  return msg_cloud_;
}


void InteractivePointCloud::clear()
{
  marker_server_.erase(marker_name_);
  marker_server_.applyChanges();
}


void InteractivePointCloud::makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud, float size)
{
  InteractiveMarker int_marker;
  int_marker.name = marker_name_;

  Marker marker;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.frame_locked = false;

  pcl::PointCloud<PointT> pcl_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  int num_points = pcl_cloud.points.size();

  if(num_points > 0)
  {
    for ( int i=0; i<num_points; i++)
    {

      int_marker.header = cloud->header;
      int_marker.header.stamp = ros::Time(0);
      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;

      //int num_points = cloud_pts_->points.size();
      marker.points.resize( num_points );
      marker.colors.resize( num_points );

      //ROS_INFO_STREAM( "Adding point cluster. #points=" << num_points << " frame=" << msg_cloud_.header.frame_id);


      marker.points[i].x = pcl_cloud.points[i].x;
      marker.points[i].y = pcl_cloud.points[i].y;
      marker.points[i].z = pcl_cloud.points[i].z;
      marker.colors[i].r = pcl_cloud.points[i].r/255.;
      marker.colors[i].g = pcl_cloud.points[i].g/255.;
      marker.colors[i].b = pcl_cloud.points[i].b/255.;
      marker.colors[i].a = 1.0;
    }
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    marker_server_.insert( int_marker, boost::bind( &InteractivePointCloud::leftClickPoint, this, _1 ),
			   visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
    menu_handler_.apply( marker_server_, marker_name_ );
    marker_server_.applyChanges();
    ROS_INFO("made interactive marker for cloud");
  }
}
