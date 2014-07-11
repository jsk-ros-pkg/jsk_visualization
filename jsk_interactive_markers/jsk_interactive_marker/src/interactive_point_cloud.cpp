#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_interactive_marker/interactive_point_cloud.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

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

  pnh_.param<double>("point_size", point_size_, 0.004);
  pnh_.param<std::string>("input", input_pointcloud_, "/selected_pointcloud");
  //  pnh_.param<bool>("use_bounding_box", use_bounding_box_, "false");
  pnh_.param<bool>("use_bounding_box", use_bounding_box_, "true");
  pnh_.param<std::string>("input_bounding_box", input_bounding_box_, "/bounding_box_marker/selected_box_array");


  pub_click_point_ = pnh_.advertise<geometry_msgs::PointStamped>("right_click_point", 1);
  pub_left_click_ = pnh_.advertise<geometry_msgs::PointStamped>("left_click_point", 1);
  pub_marker_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("marker_pose", 1);

  /*
  sub_point_cloud_ = pnh_.subscribe(input_pointcloud_, 1,
				    &InteractivePointCloud::pointCloudCallback, this);
  */
  sub_point_cloud_.subscribe(pnh_, input_pointcloud_, 1);


  if(use_bounding_box_){
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
    sub_bounding_box_.subscribe(pnh_,input_bounding_box_, 1);
    sync_->connectInput(sub_point_cloud_, sub_bounding_box_);
    sync_->registerCallback(boost::bind(&InteractivePointCloud::pointCloudAndBoundingBoxCallback, this, _1, _2));
  }else{
      sub_point_cloud_.registerCallback(&InteractivePointCloud::pointCloudCallback, this);
  }
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

 void InteractivePointCloud::pointCloudAndBoundingBoxCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const jsk_pcl_ros::BoundingBoxArrayConstPtr &box){
  ROS_INFO("%lf",point_size_);
  makeMarker(cloud, box, point_size_ );
}

void InteractivePointCloud::makeMenu()
{
  // create menu
  menu_handler_.insert( "Move",  boost::bind( &InteractivePointCloud::move, this, _1) );

  //menu_handler_.insert( "Broadcast click point",  boost::bind( &InteractivePointCloud::leftClickPoint, this, _1) );

  menu_handler_.insert( "Hide",  boost::bind( &InteractivePointCloud::hide, this, _1));
  
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
  click_point.header.stamp = ros::Time::now();
  pub_left_click_.publish(click_point);
}

void InteractivePointCloud::hide( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  marker_server_.erase(marker_name_);
  marker_server_.applyChanges();
}


void InteractivePointCloud::markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  marker_pose_.pose = feedback->pose;
  marker_pose_.header = feedback->header;
}

void InteractivePointCloud::makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud,float size){
  makeMarker(cloud, jsk_pcl_ros::BoundingBoxArray::ConstPtr(), size);
}
void InteractivePointCloud::makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud, const jsk_pcl_ros::BoundingBoxArrayConstPtr box, float size)
{
  InteractiveMarker int_marker;
  int_marker.name = marker_name_;

  pcl::PointCloud<PointT> pcl_cloud, transform_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  if(box && box->boxes.size() > 0){
    jsk_pcl_ros::BoundingBox first_box = box->boxes[0];
    int_marker.pose = first_box.pose;
    int_marker.header.frame_id = first_box.header.frame_id;

    tf::Transform transform;
    //tf::poseStampedTFToMsg();
    tf::poseMsgToTF(first_box.pose, transform);

    pcl_ros::transformPointCloud(pcl_cloud, transform_cloud, transform.inverse());
    pcl_cloud = transform_cloud;
  }


  int num_points = pcl_cloud.points.size();

  Marker marker;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.frame_locked = false;
  if(num_points > 0)
  {

      int_marker.header = cloud->header;
      marker_pose_.header = cloud->header;
      marker_pose_.pose.position.x = marker_pose_.pose.position.y = marker_pose_.pose.position.z = 0;
      marker_pose_.pose.orientation.x = 0;
      marker_pose_.pose.orientation.y = 0;
      marker_pose_.pose.orientation.z = 0;
      marker_pose_.pose.orientation.w = 1;


      InteractiveMarkerControl control;
      control.always_visible = true;
      //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
      control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

      int_marker.header.stamp = ros::Time::now();

      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;

      marker.points.resize( num_points );
      marker.colors.resize( num_points );
      //point cloud
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
      control.markers.push_back( marker );

      //bounding box
      if(box && box->boxes.size() > 0){
	Marker bounding_box_marker;
	bounding_box_marker.color.r = 0.0;
	bounding_box_marker.color.g = 1.0;
	bounding_box_marker.color.b = 0.0;
	bounding_box_marker.color.a = 1.0;
	bounding_box_marker.type = visualization_msgs::Marker::LINE_LIST;
	bounding_box_marker.scale.x = 0.01; //line width
	bounding_box_marker.points.resize(24);

	double x = box->boxes[0].dimensions.x / 2;
	double y = box->boxes[0].dimensions.y / 2;
	double z = box->boxes[0].dimensions.z / 2;
	for(int i=0; i<4; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = x;
	      
	    }else{
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = - x;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = y;
	      
	    }else{
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = - y;
	      
	    }
	    bounding_box_marker.points[2*i].z = z;
	    bounding_box_marker.points[2*i+1].z = - z;
	  }

	for(int i=4; i<8; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = x;
	      
	    }else{
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = - x;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = z;
	      
	    }else{
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = - z;
	      
	    }
	    bounding_box_marker.points[2*i].y = y;
	    bounding_box_marker.points[2*i+1].y = - y;
	  }

	for(int i=8; i<12; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = z;
	      
	    }else{
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = - z;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = y;
	      
	    }else{
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = - y;
	      
	    }
	    bounding_box_marker.points[2*i].x = x;
	    bounding_box_marker.points[2*i+1].x = - x;
	  }

	control.markers.push_back( bounding_box_marker );
      }
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
