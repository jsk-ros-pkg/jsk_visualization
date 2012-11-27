#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include <boost/foreach.hpp>

#include "wrench_visual.h"

#include "wrench_display.h"

namespace jsk_rviz_plugin
{

    WrenchStampedDisplay::WrenchStampedDisplay()
	: Display()
	, scene_node_( NULL )
	, messages_received_( 0 )
	, force_color_( .8, .2, .2 )       // Default force color is red
	, torque_color_( .8, .8, .2 )      // Default torque color is yellow
	, alpha_( 1.0 )              // Default alpha is completely opaque.
	, scale_( 2.0 )              // Default scale
	, width_( 0.5 )              // Default width 
    {
    }

    void WrenchStampedDisplay::onInitialize()
    {
	// Make an Ogre::SceneNode to contain all our visuals.
	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	// Set the default history length and resize the ``visuals_`` array.
	setHistoryLength( 1 );

	// A tf::MessageFilter listens to ROS messages and calls our
	// callback with them when they can be matched up with valid tf
	// transform data.
	tf_filter_ =
	    new tf::MessageFilter<geometry_msgs::WrenchStamped>( *vis_manager_->getTFClient(),
					     "", 100, update_nh_ );
	tf_filter_->connectInput( sub_ );
	tf_filter_->registerCallback( boost::bind( &WrenchStampedDisplay::incomingMessage,
						   this, _1 ));

	// FrameManager has some built-in functions to set the status of a
	// Display based on callbacks from a tf::MessageFilter.  These work
	// fine for this simple display.
	vis_manager_->getFrameManager()
	    ->registerFilterForTransformStatusCheck( tf_filter_, this );

    }

    WrenchStampedDisplay::~WrenchStampedDisplay()
    {
	unsubscribe();
	clear();
	visuals_.clear();

	delete tf_filter_;
    }

    // Clear the visuals by deleting their objects.
    void WrenchStampedDisplay::clear()
    {
	for( size_t i = 0; i < visuals_.size(); i++ ) {
	    delete visuals_[ i ];
	    visuals_[ i ] = NULL;
	}
	tf_filter_->clear();
	messages_received_ = 0;
	setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
    }

    void WrenchStampedDisplay::setTopic( const std::string& topic )
    {
	unsubscribe();
	clear();
	topic_ = topic;
	subscribe();

	// Broadcast the fact that the variable has changed.
	propertyChanged( topic_property_ );

	// Make sure rviz renders the next time it gets a chance.
	causeRender();
    }

    void WrenchStampedDisplay::setForceColor( const rviz::Color& color )
    {
	force_color_ = color;

	propertyChanged( force_color_property_ );
	updateVisual();
	causeRender();
    }

    void WrenchStampedDisplay::setTorqueColor( const rviz::Color& color )
    {
	torque_color_ = color;

	propertyChanged( torque_color_property_ );
	updateVisual();
	causeRender();
    }

    void WrenchStampedDisplay::setAlpha( float alpha )
    {
	alpha_ = alpha;

	propertyChanged( alpha_property_ );
	updateVisual();
	causeRender();
    }

    void WrenchStampedDisplay::setScale( float scale )
    {
	scale_ = scale;

	propertyChanged( width_property_ );
	updateVisual();
	causeRender();
    }

    void WrenchStampedDisplay::setWidth( float width )
    {
	width_ = width;

	propertyChanged( width_property_ );
	updateVisual();
	causeRender();
    }

    // Set the current color and alpha values for each visual.
    void WrenchStampedDisplay::updateVisual()
    {
	BOOST_FOREACH(MapWrenchStampedVisual::value_type visual, visuals_)
	{
            if ( visual ) {
                visual->setForceColor( force_color_.r_, force_color_.g_, force_color_.b_, alpha_ );
                visual->setTorqueColor( torque_color_.r_, torque_color_.g_, torque_color_.b_, alpha_ );            
                visual->setScale( scale_ );
                visual->setWidth( width_ );
            }
	}
    }

    // Set the number of past visuals to show.
    void WrenchStampedDisplay::setHistoryLength( int length )
    {
	// Don't let people enter invalid values.
	if( length < 1 )
	{
	    length = 1;
	}
	// If the length is not changing, we don't need to do anything.
	if( history_length_ == length )
	{
	    return;
	}

	// Set the actual variable.
	history_length_ = length;
	propertyChanged( history_length_property_ );

	// Create a new array of visual pointers, all NULL.
	std::vector<WrenchStampedVisual*> new_visuals( history_length_, (WrenchStampedVisual*)0 );

	// Copy the contents from the old array to the new.
	// (Number to copy is the minimum of the 2 vector lengths).
	size_t copy_len =
	    (new_visuals.size() > visuals_.size()) ?
	    visuals_.size() : new_visuals.size();
	for( size_t i = 0; i < copy_len; i++ )
	{
	    int new_index = (messages_received_ - i) % new_visuals.size();
	    int old_index = (messages_received_ - i) % visuals_.size();
	    new_visuals[ new_index ] = visuals_[ old_index ];
	    visuals_[ old_index ] = NULL;
	}

	// Delete any remaining old visuals
	for( size_t i = 0; i < visuals_.size(); i++ ) {
	    delete visuals_[ i ];
	}

	// We don't need to create any new visuals here, they are created as
	// needed when messages are received.

	// Put the new vector into the member variable version and let the
	// old one go out of scope.
	visuals_.swap( new_visuals );
    }

    void WrenchStampedDisplay::subscribe()
    {
	// If we are not actually enabled, don't do it.
	if ( !isEnabled() )
	{
	    return;
	}

	// Try to subscribe to the current topic name (in ``topic_``).  Make
	// sure to catch exceptions and set the status to a descriptive
	// error message.
	try
	{
	    sub_.subscribe( update_nh_, topic_, 10 );
	    setStatus( rviz::status_levels::Ok, "Topic", "OK" );
	}
	catch( ros::Exception& e )
	{
	    setStatus( rviz::status_levels::Error, "Topic",
		       std::string( "Error subscribing: " ) + e.what() );
	}
    }

    void WrenchStampedDisplay::unsubscribe()
    {
	sub_.unsubscribe();
    }

    void WrenchStampedDisplay::onEnable()
    {
	subscribe();
    }

    void WrenchStampedDisplay::onDisable()
    {
	unsubscribe();
	clear();
    }

    // When the "Fixed Frame" changes, we need to update our
    // tf::MessageFilter and erase existing visuals.
    void WrenchStampedDisplay::fixedFrameChanged()
    {
	tf_filter_->setTargetFrame( fixed_frame_ );
	clear();
    }

    // This is our callback to handle an incoming message.
    void WrenchStampedDisplay::incomingMessage( const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
	++messages_received_;

	// Each display can have multiple status lines.  This one is called
	// "Topic" and says how many messages have been received in this case.
	std::stringstream ss;
	ss << messages_received_ << " messages received";
	setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Imu message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if( !vis_manager_->getFrameManager()->getTransform( msg->header.frame_id,
							    msg->header.stamp,
							    position, orientation ))
	  {
	    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		       msg->header.frame_id.c_str(), fixed_frame_.c_str() );
	    return;
	  }

	// We are keeping a circular buffer of visual pointers.  This gets
	// the next one, or creates and stores it if it was missing.
	WrenchStampedVisual* visual = visuals_[ messages_received_ % history_length_ ];
	if( visual == NULL )
	  {
	    visual = new WrenchStampedVisual( vis_manager_->getSceneManager(), scene_node_ );
	    visuals_[ messages_received_ % history_length_ ] = visual;
	  }

	// Now set or update the contents of the chosen visual.
	visual->setMessage( msg );
	visual->setFramePosition( position );
	visual->setFrameOrientation( orientation );
	visual->setForceColor( force_color_.r_, force_color_.g_, force_color_.b_, alpha_ );
	visual->setTorqueColor( torque_color_.r_, torque_color_.g_, torque_color_.b_, alpha_ );
        visual->setScale( scale_ );
        visual->setWidth( width_ );
    }

    // Override rviz::Display's reset() function to add a call to clear().
    void WrenchStampedDisplay::reset()
    {
	Display::reset();
	clear();
    }

    void WrenchStampedDisplay::createProperties()
    {
	topic_property_ =
	    property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
									     property_prefix_,
									     boost::bind( &WrenchStampedDisplay::getTopic, this ),
									     boost::bind( &WrenchStampedDisplay::setTopic, this, _1 ),
									     parent_category_,
									     this );
	setPropertyHelpText( topic_property_, "sensor_msgs::WrenchStamped topic to subscribe to." );
	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType( ros::message_traits::datatype<geometry_msgs::WrenchStamped>() );

	force_color_property_ =
	    property_manager_->createProperty<rviz::ColorProperty>( "Force Color",
								    property_prefix_,
								    boost::bind( &WrenchStampedDisplay::getForceColor, this ),
								    boost::bind( &WrenchStampedDisplay::setForceColor, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( force_color_property_, "Color to draw the force arrows." );

	torque_color_property_ =
	    property_manager_->createProperty<rviz::ColorProperty>( "Torque Color",
								    property_prefix_,
								    boost::bind( &WrenchStampedDisplay::getTorqueColor, this ),
								    boost::bind( &WrenchStampedDisplay::setTorqueColor, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( torque_color_property_, "Color to draw the torque arrows." );

	alpha_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Alpha",
								    property_prefix_,
								    boost::bind( &WrenchStampedDisplay::getAlpha, this ),
								    boost::bind( &WrenchStampedDisplay::setAlpha, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( alpha_property_, "0 is fully transparent, 1.0 is fully opaque." );

	scale_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Arrow Scale",
								    property_prefix_,
								    boost::bind( &WrenchStampedDisplay::getScale, this ),
								    boost::bind( &WrenchStampedDisplay::setScale, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( alpha_property_, "arrow scale" );


	width_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Arrow Width",
								    property_prefix_,
								    boost::bind( &WrenchStampedDisplay::getWidth, this ),
								    boost::bind( &WrenchStampedDisplay::setWidth, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( alpha_property_, "arrow width" );


	history_length_property_ =
	    property_manager_->createProperty<rviz::IntProperty>( "History Length",
								  property_prefix_,
								  boost::bind( &WrenchStampedDisplay::getHistoryLength, this ),
								  boost::bind( &WrenchStampedDisplay::setHistoryLength, this, _1 ),
								  parent_category_,
							    this );
	setPropertyHelpText( history_length_property_, "Number of prior measurements to display." );
    }
} // end namespace jsk_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( jsk_rviz_plugin, WrenchStamped, jsk_rviz_plugin::WrenchStampedDisplay, rviz::Display )


