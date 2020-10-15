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
#include <pcl_conversions/pcl_conversions.h>

#include <boost/range/algorithm_ext/erase.hpp>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace im_helpers;

InteractivePointCloud::InteractivePointCloud(std::string marker_name,
			   std::string topic_name, std::string server_name ):
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
  pnh_.param<std::string>("handle_pose", initial_handle_pose_, "/handle_estimator/output_best");
  pnh_.param<bool>("display_interactive_manipulator", display_interactive_manipulator_, true);
  //publish
  pub_click_point_ = pnh_.advertise<geometry_msgs::PointStamped>("right_click_point", 1);
  pub_left_click_ = pnh_.advertise<geometry_msgs::PointStamped>("left_click_point", 1);
  pub_left_click_relative_ = pnh_.advertise<geometry_msgs::PointStamped>("left_click_point_relative", 1);
  pub_marker_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("marker_pose", 1);
  pub_grasp_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("grasp_pose", 1);
  pub_box_movement_ = pnh_.advertise<jsk_recognition_msgs::BoundingBoxMovement>("box_movement", 1);
  pub_handle_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("handle_pose", 1);
  pub_handle_pose_array_ = pnh_.advertise<geometry_msgs::PoseArray>("handle_pose_array", 1);
  
  //subscribe
  sub_handle_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped> ("set_handle_pose", 1, boost::bind( &InteractivePointCloud::setHandlePoseCallback, this, _1));
  sub_marker_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped>("set_marker_pose", 1, boost::bind( &InteractivePointCloud::setMarkerPoseCallback, this, _1)); 
  sub_point_cloud_.subscribe(pnh_, input_pointcloud_, 1);


  if(use_bounding_box_){
    //point cloud and bounding box
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
    sub_bounding_box_.subscribe(pnh_,input_bounding_box_, 1);
    sub_initial_handle_pose_.subscribe(pnh_, initial_handle_pose_, 1);
    sync_->connectInput(sub_point_cloud_, sub_bounding_box_, sub_initial_handle_pose_);
    sync_->registerCallback(boost::bind(&InteractivePointCloud::pointCloudAndBoundingBoxCallback, this, _1, _2, _3));

  }else{
      sub_point_cloud_.registerCallback(&InteractivePointCloud::pointCloudCallback, this);
  }
  srv_ = std::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
    boost::bind (&InteractivePointCloud::configCallback, this, _1, _2);
  srv_->setCallback (f);

  makeMenu();
}

InteractivePointCloud::~InteractivePointCloud(){};

void InteractivePointCloud::configCallback(Config &config, uint32_t level)
{
  point_size_ = config.point_size;
  if(display_interactive_manipulator_ != config.display_interactive_manipulator){
    display_interactive_manipulator_ = config.display_interactive_manipulator;
    visualization_msgs::InteractiveMarker int_marker;
    marker_server_.get(marker_name_, int_marker );
    if(display_interactive_manipulator_){
      addVisible6DofControl(int_marker);
    }else{
      for(std::vector<visualization_msgs::InteractiveMarkerControl>::iterator it=int_marker.controls.begin(); it!=int_marker.controls.end();){	
      	if(it->interaction_mode==visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS
      	   || it->interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_AXIS){
	  it = int_marker.controls.erase(it);
      	}else{
	  it++;
	}
      }
    }
    marker_server_.erase(marker_name_);
    marker_server_.insert(int_marker);
    marker_server_.applyChanges();
  }
}

void InteractivePointCloud::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud){
  makeMarker(cloud, point_size_ );
}

void InteractivePointCloud::pointCloudAndBoundingBoxCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const jsk_recognition_msgs::BoundingBoxArrayConstPtr &box, const geometry_msgs::PoseStampedConstPtr &handle){
  makeMarker(cloud, box, handle, point_size_ );
}



void InteractivePointCloud::setHandlePoseCallback(const geometry_msgs::PoseStampedConstPtr &ps){
  if( ps->header.stamp == current_box_.header.stamp ){
    if(current_box_.boxes.size() > 0){
      handle_pose_ = *ps;
      
      tf::Transform tf_ps, tf_box;
      tf_ps.setOrigin(tf::Vector3(ps->pose.position.x, ps->pose.position.y, ps->pose.position.z));
      tf_ps.setRotation(tf::Quaternion( ps->pose.orientation.x, ps->pose.orientation.y, ps->pose.orientation.z, ps->pose.orientation.w));

      geometry_msgs::Pose pose = current_box_.boxes[0].pose;
      tf_box.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
      tf_box.setRotation(tf::Quaternion( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

      handle_tf_ = tf_box.inverseTimes(tf_ps);
      exist_handle_tf_ = true;
      publishHandPose( marker_pose_ );

      /* set box_movement_ */
      geometry_msgs::Pose handle_pose;
      handle_pose.position.x = handle_tf_.getOrigin().x();
      handle_pose.position.y = handle_tf_.getOrigin().y();
      handle_pose.position.z = handle_tf_.getOrigin().z();
      handle_pose.orientation.x = handle_tf_.getRotation().x();
      handle_pose.orientation.y = handle_tf_.getRotation().y();
      handle_pose.orientation.z = handle_tf_.getRotation().z();
      handle_pose.orientation.w = handle_tf_.getRotation().w();

      box_movement_.handle_pose = handle_pose;
    }
  }
}

void InteractivePointCloud::setMarkerPoseCallback( const geometry_msgs::PoseStampedConstPtr &pose_stamped_msg){
  marker_server_.setPose(marker_name_, pose_stamped_msg->pose, pose_stamped_msg->header);
  marker_server_.applyChanges();
}

// create menu
void InteractivePointCloud::makeMenu()
{
  menu_handler_.insert( "Move",  boost::bind( &InteractivePointCloud::move, this, _1) );

  menu_handler_.insert( "Hide",  boost::bind( &InteractivePointCloud::hide, this, _1));
}

//publish marker pose
void InteractivePointCloud::move( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  box_movement_.destination = marker_pose_;
  pub_box_movement_.publish(box_movement_);

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
  tf::Transform transform;
  tf::poseMsgToTF(feedback->pose, transform);
  tf::Vector3 vector_absolute(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z); 
  tf::Vector3 vector_relative=transform.inverse() * vector_absolute;
  click_point.point.x=vector_relative.getX();
  click_point.point.y=vector_relative.getY();   
  click_point.point.z=vector_relative.getZ();
  pub_left_click_relative_.publish(click_point);
}

void InteractivePointCloud::hide( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  marker_server_.erase(marker_name_);
  marker_server_.applyChanges();
}


void InteractivePointCloud::markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  marker_pose_.pose = feedback->pose;
  marker_pose_.header = feedback->header;
  if(exist_handle_tf_){
    publishHandPose( marker_pose_);
  }
}

void InteractivePointCloud::publishGraspPose(){
  pub_grasp_pose_.publish(handle_pose_);
}

void InteractivePointCloud::publishHandPose( geometry_msgs::PoseStamped box_pose){
  tf::Transform tf_marker;
  tf_marker.setOrigin(tf::Vector3(box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z));
  tf_marker.setRotation(tf::Quaternion( box_pose.pose.orientation.x, box_pose.pose.orientation.y, box_pose.pose.orientation.z, box_pose.pose.orientation.w));

  tf_marker = tf_marker * handle_tf_;
  geometry_msgs::PoseStamped handle_pose;
  handle_pose.header.frame_id = box_pose.header.frame_id;
  handle_pose.pose.position.x = tf_marker.getOrigin().x();
  handle_pose.pose.position.y = tf_marker.getOrigin().y();
  handle_pose.pose.position.z = tf_marker.getOrigin().z();
  handle_pose.pose.orientation.x = tf_marker.getRotation().x();
  handle_pose.pose.orientation.y = tf_marker.getRotation().y();
  handle_pose.pose.orientation.z = tf_marker.getRotation().z();
  handle_pose.pose.orientation.w = tf_marker.getRotation().w();

  pub_handle_pose_.publish(handle_pose);

  geometry_msgs::PoseArray handle_pose_array;
  handle_pose_array.header = handle_pose.header;
  handle_pose_array.poses.push_back(handle_pose.pose);
  pub_handle_pose_array_.publish(handle_pose_array);
}

void InteractivePointCloud::makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud,float size){
  makeMarker(cloud, jsk_recognition_msgs::BoundingBoxArray::ConstPtr(), geometry_msgs::PoseStamped::ConstPtr(), size);
}
void InteractivePointCloud::makeMarker(const sensor_msgs::PointCloud2ConstPtr cloud, const jsk_recognition_msgs::BoundingBoxArrayConstPtr box, const geometry_msgs::PoseStampedConstPtr handle, float size)
{
  exist_handle_tf_ = false;
  if(cloud){
    current_croud_ = *cloud;
  }
  if(box){
    current_box_ = *box;
  }

  InteractiveMarker int_marker;
  int_marker.name = marker_name_;

  pcl::PointCloud<PointT> pcl_cloud, transform_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  if(box && box->boxes.size() > 0){
    jsk_recognition_msgs::BoundingBox first_box = box->boxes[0];
    int_marker.pose = first_box.pose;
    int_marker.header.frame_id = first_box.header.frame_id;

    marker_pose_.header = box->header;
    marker_pose_.pose = first_box.pose;

    tf::Transform transform;
    tf::poseMsgToTF(first_box.pose, transform);

    pcl_ros::transformPointCloud(pcl_cloud, transform_cloud, transform.inverse());
    pcl_cloud = transform_cloud;

    box_movement_.box = first_box;
    box_movement_.header.frame_id = first_box.header.frame_id;

    if(handle->header.frame_id != ""){
      setHandlePoseCallback(handle);
    }
  }
  else{
    int_marker.pose.position.x=int_marker.pose.position.y=int_marker.pose.position.z=int_marker.pose.orientation.x=int_marker.pose.orientation.y=int_marker.pose.orientation.z=0;
    int_marker.pose.orientation.w=1;
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

      InteractiveMarkerControl control;
      control.always_visible = true;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
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
	bounding_box_marker.color.g = 0.0;
	bounding_box_marker.color.b = 1.0;
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
      if(display_interactive_manipulator_){
	addVisible6DofControl(int_marker);
      }
      marker_server_.insert( int_marker, boost::bind( &InteractivePointCloud::leftClickPoint, this, _1 ),
			     visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
      marker_server_.setCallback( int_marker.name,
				  boost::bind( &InteractivePointCloud::markerFeedback, this, _1) );

      menu_handler_.apply( marker_server_, marker_name_ );
      marker_server_.applyChanges();
      ROS_INFO("made interactive point cloud");
      
      publishGraspPose();
  }
}
