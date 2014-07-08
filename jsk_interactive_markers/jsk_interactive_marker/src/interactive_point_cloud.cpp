#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_interactive_marker/interactive_point_cloud.h>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace im_helpers;

InteractivePointCloud::InteractivePointCloud(std::string marker_name,
			   std::string topic_name, std::string server_name
					     ):
  marker_name_(marker_name)
  , nh_()
  , pnh_("~")
  , tfl_(nh_)
  , marker_server_(topic_name, server_name, false)
{

  pnh_.param<double>("point_size", point_size_, 0.002);
  pnh_.param<std::string>("input", input_pointcloud_, "/selected_pointcloud");

  pub_click_point_ = pnh_.advertise<geometry_msgs::PointStamped>("right_click_point", 1);
  pub_left_click_ = pnh_.advertise<geometry_msgs::PointStamped>("left_click_point", 1);
  pub_marker_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("marker_pose", 1);

  sub_point_cloud_ = pnh_.subscribe(input_pointcloud_, 1,
				    &InteractivePointCloud::pointCloudCallback, this);

  srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
    boost::bind (&InteractivePointCloud::configCallback, this, _1, _2);
  srv_->setCallback (f);

  makeMenu();
}

InteractivePointCloud::~InteractivePointCloud(){};

void InteractivePointCloud::configCallback(Config &config, uint32_t level)
{
  boost::mutex::scoped_lock(mutex_);
  point_size_ = config.point_size;
}



void InteractivePointCloud::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud){
  ROS_INFO("%lf",point_size_);
  makeMarker(cloud, point_size_ );
}

void InteractivePointCloud::makeMenu()
{
  // create menu
  menu_handler_.insert( "Move",  boost::bind( &InteractivePointCloud::move, this, _1) );

  menu_handler_.insert( "Broadcast click point",  boost::bind( &InteractivePointCloud::leftClickPoint, this, _1) );

  menu_handler_.insert( "Clear",    boost::bind( &InteractivePointCloud::clear, this) );
}


void InteractivePointCloud::move( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  pub_marker_pose_.publish(marker_pose_);

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
  pub_left_click_.publish(click_point);
}

void InteractivePointCloud::clear()
{
  marker_server_.erase(marker_name_);
  marker_server_.applyChanges();
}


void InteractivePointCloud::markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  marker_pose_.pose = feedback->pose;
  marker_pose_.header = feedback->header;
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

      int_marker.header = cloud->header;
      marker_pose_.header = cloud->header;
      marker_pose_.pose.position.x = marker_pose_.pose.position.y = marker_pose_.pose.position.z = 0;
      marker_pose_.pose.orientation.x = 0;
      marker_pose_.pose.orientation.y = 0;
      marker_pose_.pose.orientation.z = 0;
      marker_pose_.pose.orientation.w = 1;

      int_marker.header.stamp = ros::Time(0);
      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;

      marker.points.resize( num_points );
      marker.colors.resize( num_points );
      for ( int i=0; i<num_points; i++)
	{
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
    
    addVisible6DofControl(int_marker);

    marker_server_.insert( int_marker, boost::bind( &InteractivePointCloud::leftClickPoint, this, _1 ),
			   visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
    marker_server_.setCallback( int_marker.name,
				boost::bind( &InteractivePointCloud::markerFeedback, this, _1) );

    menu_handler_.apply( marker_server_, marker_name_ );
    marker_server_.applyChanges();
    ROS_INFO("made interactive point cloud");
  }
}
