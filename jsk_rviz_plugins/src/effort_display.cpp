#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include <boost/foreach.hpp>

#include "effort_visual.h"

#include "effort_display.h"

#include <urdf/model.h>

// MessageFilter to support message with out frame_id
// https://code.ros.org/trac/ros-pkg/ticket/5467
namespace tf {

#define TF_MESSAGEFILTER_DEBUG(fmt, ...) \
  ROS_DEBUG_NAMED("message_filter", "MessageFilter [target=%s]: "fmt, getTargetFramesString().c_str(), __VA_ARGS__)

#define TF_MESSAGEFILTER_WARN(fmt, ...) \
  ROS_WARN_NAMED("message_filter", "MessageFilter [target=%s]: "fmt, getTargetFramesString().c_str(), __VA_ARGS__)

    class MessageFilterJointState : public MessageFilter<sensor_msgs::JointState>
    {
	typedef sensor_msgs::JointState M;
public:
	typedef boost::shared_ptr<M const> MConstPtr;
	typedef ros::MessageEvent<M const> MEvent;
	typedef boost::function<void(const MConstPtr&, FilterFailureReason)> FailureCallback;
	typedef boost::signal<void(const MConstPtr&, FilterFailureReason)> FailureSignal;

	// If you hit this assert your message does not have a header, or does not have the HasHeader trait defined for it
	ROS_STATIC_ASSERT(ros::message_traits::HasHeader<M>::value);

	/**
	 * \brief Constructor
	 *
	 * \param tf The tf::Transformer this filter should use
	 * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
	 * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
	 * \param nh The NodeHandle to use for any necessary operations
	 * \param max_rate The maximum rate to check for newly transformable messages
	 */
	MessageFilterJointState(Transformer& tf, const std::string& target_frame, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle(), ros::Duration max_rate = ros::Duration(0.01))
	    : MessageFilter<sensor_msgs::JointState>(tf, target_frame, queue_size, nh, max_rate)
            , tf_(tf)
	    , nh_(nh)
	    , max_rate_(max_rate)
	    , queue_size_(queue_size)
	    {
		init();

		setTargetFrame(target_frame);
	    }

	/**
	 * \brief Constructor
	 *
	 * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
	 * \param tf The tf::Transformer this filter should use
	 * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
	 * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
	 * \param nh The NodeHandle to use for any necessary operations
	 * \param max_rate The maximum rate to check for newly transformable messages
	 */
	template<class F>
	MessageFilterJointState(F& f, Transformer& tf, const std::string& target_frame, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle(), ros::Duration max_rate = ros::Duration(0.01))
	    : tf_(tf)
	    , nh_(nh)
	    , max_rate_(max_rate)
	    , queue_size_(queue_size)
            , MessageFilter<sensor_msgs::JointState>(f, tf, target_frame, queue_size, nh, max_rate)
	    {
		init();

		setTargetFrame(target_frame);

		connectInput(f);
	    }

	/**
	 * \brief Connect this filter's input to another filter's output.  If this filter is already connected, disconnects first.
	 */
	template<class F>
	void connectInput(F& f)
	    {
		message_connection_.disconnect();
		message_connection_ = f.registerCallback(&MessageFilterJointState::incomingMessage, this);
	    }

	/**
	 * \brief Destructor
	 */
	~MessageFilterJointState()
	    {
		message_connection_.disconnect();
		tf_.removeTransformsChangedListener(tf_connection_);

		clear();

		TF_MESSAGEFILTER_DEBUG("Successful Transforms: %llu, Failed Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, Messages received: %llu, Total dropped: %llu",
				       (long long unsigned int)successful_transform_count_, (long long unsigned int)failed_transform_count_, 
				       (long long unsigned int)failed_out_the_back_count_, (long long unsigned int)transform_message_count_, 
				       (long long unsigned int)incoming_message_count_, (long long unsigned int)dropped_message_count_);

	    }

	/**
	 * \brief Set the frame you need to be able to transform to before getting a message callback
	 */
	void setTargetFrame(const std::string& target_frame)
	    {
		std::vector<std::string> frames;
		frames.push_back(target_frame);
		setTargetFrames(frames);
	    }

	/**
	 * \brief Set the frames you need to be able to transform to before getting a message callback
	 */
	void setTargetFrames(const std::vector<std::string>& target_frames)
	    {
		boost::mutex::scoped_lock list_lock(messages_mutex_);
		boost::mutex::scoped_lock string_lock(target_frames_string_mutex_);

		target_frames_ = target_frames;

		std::stringstream ss;
		for (std::vector<std::string>::iterator it = target_frames_.begin(); it != target_frames_.end(); ++it)
		{
		    *it = tf::resolve(tf_.getTFPrefix(), *it);
		    ss << *it << " ";
		}
		target_frames_string_ = ss.str();
	    }
	/**
	 * \brief Get the target frames as a string for debugging
	 */
	std::string getTargetFramesString()
	    {
		boost::mutex::scoped_lock lock(target_frames_string_mutex_);
		return target_frames_string_;
	    };

	/**
	 * \brief Set the required tolerance for the notifier to return true
	 */
	void setTolerance(const ros::Duration& tolerance)
	    {
		time_tolerance_ = tolerance;
	    }

	/**
	 * \brief Clear any messages currently in the queue
	 */
	void clear()
	    {
		boost::mutex::scoped_lock lock(messages_mutex_);

		TF_MESSAGEFILTER_DEBUG("%s", "Cleared");

		messages_.clear();
		message_count_ = 0;

		warned_about_unresolved_name_ = false;
		warned_about_empty_frame_id_ = false;
	    }

	void add(const MEvent& evt)
	    {
		boost::mutex::scoped_lock lock(messages_mutex_);

		testMessages();

		if (!testMessage(evt))
		{
		    // If this message is about to push us past our queue size, erase the oldest message
		    if (queue_size_ != 0 && message_count_ + 1 > queue_size_)
		    {
			++dropped_message_count_;
			const MEvent& front = messages_.front();
			TF_MESSAGEFILTER_DEBUG("Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)", message_count_, front.getMessage()->header.frame_id.c_str(), front.getMessage()->header.stamp.toSec());
			signalFailure(messages_.front(), filter_failure_reasons::Unknown);

			messages_.pop_front();
			--message_count_;
		    }

		    // Add the message to our list
		    messages_.push_back(evt);
		    ++message_count_;
		}

		TF_MESSAGEFILTER_DEBUG("Added message in frame %s at time %.3f, count now %d", evt.getMessage()->header.frame_id.c_str(), evt.getMessage()->header.stamp.toSec(), message_count_);

		++incoming_message_count_;
	    }

	/**
	 * \brief Manually add a message into this filter.
	 * \note If the message (or any other messages in the queue) are immediately transformable this will immediately call through to the output callback, possibly
	 * multiple times
	 */
	void add(const MConstPtr& message)
	    {
		boost::shared_ptr<std::map<std::string, std::string> > header(new std::map<std::string, std::string>);
		(*header)["callerid"] = "unknown";
		add(MEvent(message, header, ros::Time::now()));
	    }

	/**
	 * \brief Register a callback to be called when a message is about to be dropped
	 * \param callback The callback to call
	 */
	message_filters::Connection registerFailureCallback(const FailureCallback& callback)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		return message_filters::Connection(boost::bind(&MessageFilterJointState::disconnectFailure, this, _1), failure_signal_.connect(callback));
	    }

	virtual void setQueueSize( uint32_t new_queue_size )
	    {
		queue_size_ = new_queue_size;
	    }

	virtual uint32_t getQueueSize()
	    {
		return queue_size_;
	    }

    private:

	void init()
	    {
		message_count_ = 0;
		new_transforms_ = false;
		successful_transform_count_ = 0;
		failed_transform_count_ = 0;
		failed_out_the_back_count_ = 0;
		transform_message_count_ = 0;
		incoming_message_count_ = 0;
		dropped_message_count_ = 0;
		time_tolerance_ = ros::Duration(0.0);
		warned_about_unresolved_name_ = false;
		warned_about_empty_frame_id_ = false;

		tf_connection_ = tf_.addTransformsChangedListener(boost::bind(&MessageFilterJointState::transformsChanged, this));

		max_rate_timer_ = nh_.createTimer(max_rate_, &MessageFilterJointState::maxRateTimerCallback, this);
	    }

	typedef std::list<MEvent> L_Event;

	bool testMessage(const MEvent& evt)
	    {
		const MConstPtr& message = evt.getMessage();
		std::string callerid = evt.getPublisherName();//message->__connection_header ? (*message->__connection_header)["callerid"] : "unknown";
		std::string frame_id = ros::message_traits::FrameId<M>::value(*message);
		ros::Time stamp = ros::message_traits::TimeStamp<M>::value(*message);

		// FIXED
		if (frame_id.empty()) frame_id = * (target_frames_.begin());
		//Throw out messages which have an empty frame_id
		if (frame_id.empty())
		{
		    if (!warned_about_empty_frame_id_)
		    {
			warned_about_empty_frame_id_ = true;
			TF_MESSAGEFILTER_WARN("Discarding message from [%s] due to empty frame_id.  This message will only print once.", callerid.c_str());
		    }
		    signalFailure(evt, filter_failure_reasons::EmptyFrameID);
		    return true;
		}

		if (frame_id[0] != '/')
		{
		    std::string unresolved = frame_id;
		    frame_id = tf::resolve(tf_.getTFPrefix(), frame_id);

		    if (!warned_about_unresolved_name_)
		    {
			warned_about_unresolved_name_ = true;
			ROS_WARN("Message from [%s] has a non-fully-qualified frame_id [%s]. Resolved locally to [%s].  This is will likely not work in multi-robot systems.  This message will only print once.", callerid.c_str(), unresolved.c_str(), frame_id.c_str());
		    }
		}

		//Throw out messages which are too old
		//! \todo combine getLatestCommonTime call with the canTransform call
		for (std::vector<std::string>::iterator target_it = target_frames_.begin(); target_it != target_frames_.end(); ++target_it)
		{
		    const std::string& target_frame = *target_it;

		    if (target_frame != frame_id && stamp != ros::Time(0))
		    {
			ros::Time latest_transform_time ;

			tf_.getLatestCommonTime(frame_id, target_frame, latest_transform_time, 0) ;
			if (stamp + tf_.getCacheLength() < latest_transform_time)
			{
			    ++failed_out_the_back_count_;
			    ++dropped_message_count_;
			    TF_MESSAGEFILTER_DEBUG("Discarding Message, in frame %s, Out of the back of Cache Time(stamp: %.3f + cache_length: %.3f < latest_transform_time %.3f.  Message Count now: %d", message->header.frame_id.c_str(), message->header.stamp.toSec(),  tf_.getCacheLength().toSec(), latest_transform_time.toSec(), message_count_);

			    last_out_the_back_stamp_ = stamp;
			    last_out_the_back_frame_ = frame_id;

			    signalFailure(evt, filter_failure_reasons::OutTheBack);
			    return true;
			}
		    }

		}

		bool ready = !target_frames_.empty();
		for (std::vector<std::string>::iterator target_it = target_frames_.begin(); ready && target_it != target_frames_.end(); ++target_it)
		{
		    std::string& target_frame = *target_it;
		    if (time_tolerance_ != ros::Duration(0.0))
		    {
			ready = ready && (tf_.canTransform(target_frame, frame_id, stamp) &&
					  tf_.canTransform(target_frame, frame_id, stamp + time_tolerance_) );
		    }
		    else
		    {
			ready = ready && tf_.canTransform(target_frame, frame_id, stamp);
		    }
		}

		if (ready)
		{
		    TF_MESSAGEFILTER_DEBUG("Message ready in frame %s at time %.3f, count now %d", frame_id.c_str(), stamp.toSec(), message_count_);

		    ++successful_transform_count_;

		    signalMessage(evt);
		}
		else
		{
		    ++failed_transform_count_;
		}

		return ready;
	    }

	void testMessages()
	    {
		if (!messages_.empty() && getTargetFramesString() == " ")
		{
		    ROS_WARN_NAMED("message_notifier", "MessageFilter [target=%s]: empty target frame", getTargetFramesString().c_str());
		}

		int i = 0;

		L_Event::iterator it = messages_.begin();
		for (; it != messages_.end(); ++i)
		{
		    MEvent& evt = *it;

		    if (testMessage(evt))
		    {
			--message_count_;
			it = messages_.erase(it);
		    }
		    else
		    {
			++it;
		    }
		}
	    }

	void maxRateTimerCallback(const ros::TimerEvent&)
	    {
		boost::mutex::scoped_lock list_lock(messages_mutex_);
		if (new_transforms_)
		{
		    testMessages();
		    new_transforms_ = false;
		}

		checkFailures();
	    }

	/**
	 * \brief Callback that happens when we receive a message on the message topic
	 */
	void incomingMessage(const ros::MessageEvent<M const>& evt)
	    {
		add(evt);
	    }

	void transformsChanged()
	    {
		new_transforms_ = true;

		++transform_message_count_;
	    }

	void checkFailures()
	    {
		if (next_failure_warning_.isZero())
		{
		    next_failure_warning_ = ros::Time::now() + ros::Duration(15);
		}

		if (ros::Time::now() >= next_failure_warning_)
		{
		    if (incoming_message_count_ - message_count_ == 0)
		    {
			return;
		    }

		    double dropped_pct = (double)dropped_message_count_ / (double)(incoming_message_count_ - message_count_);
		    if (dropped_pct > 0.95)
		    {
			TF_MESSAGEFILTER_WARN("Dropped %.2f%% of messages so far. Please turn the [%s.message_notifier] rosconsole logger to DEBUG for more information.", dropped_pct*100, ROSCONSOLE_DEFAULT_NAME);
			next_failure_warning_ = ros::Time::now() + ros::Duration(60);

			if ((double)failed_out_the_back_count_ / (double)dropped_message_count_ > 0.5)
			{
			    TF_MESSAGEFILTER_WARN("  The majority of dropped messages were due to messages growing older than the TF cache time.  The last message's timestamp was: %f, and the last frame_id was: %s", last_out_the_back_stamp_.toSec(), last_out_the_back_frame_.c_str());
			}
		    }
		}
	    }

	void disconnectFailure(const message_filters::Connection& c)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		c.getBoostConnection().disconnect();
	    }

	void signalFailure(const MEvent& evt, FilterFailureReason reason)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		failure_signal_(evt.getMessage(), reason);
	    }

	Transformer& tf_; ///< The Transformer used to determine if transformation data is available
	ros::NodeHandle nh_; ///< The node used to subscribe to the topic
	ros::Duration max_rate_;
	ros::Timer max_rate_timer_;
	std::vector<std::string> target_frames_; ///< The frames we need to be able to transform to before a message is ready
	std::string target_frames_string_;
	boost::mutex target_frames_string_mutex_;
	uint32_t queue_size_; ///< The maximum number of messages we queue up

	L_Event messages_; ///< The message list
	uint32_t message_count_; ///< The number of messages in the list.  Used because messages_.size() has linear cost
	boost::mutex messages_mutex_; ///< The mutex used for locking message list operations

	bool new_messages_; ///< Used to skip waiting on new_data_ if new messages have come in while calling back
	volatile bool new_transforms_; ///< Used to skip waiting on new_data_ if new transforms have come in while calling back or transforming data

	bool warned_about_unresolved_name_;
	bool warned_about_empty_frame_id_;

	uint64_t successful_transform_count_;
	uint64_t failed_transform_count_;
	uint64_t failed_out_the_back_count_;
	uint64_t transform_message_count_;
	uint64_t incoming_message_count_;
	uint64_t dropped_message_count_;

	ros::Time last_out_the_back_stamp_;
	std::string last_out_the_back_frame_;

	ros::Time next_failure_warning_;

	ros::Duration time_tolerance_; ///< Provide additional tolerance on time for messages which are stamped but can have associated duration

	boost::signals::connection tf_connection_;
	message_filters::Connection message_connection_;

	FailureSignal failure_signal_;
	boost::mutex failure_signal_mutex_;
    };
}

namespace jsk_rviz_plugin
{

    struct JointInfo {
        JointInfo();

        bool isEnabled() { return enabled_; }
        void setEffort(double e) { effort_ = e; }
        double getEffort() { return effort_; }
        void setMaxEffort(double m) { max_effort_ = m; }
        double getMaxEffort() { return max_effort_; }

        std::string name_;
        bool enabled_;
        double effort_, max_effort_;

        ros::Time last_update_;

        rviz::CategoryPropertyWPtr category_;
        rviz::BoolPropertyWPtr enabled_property_;
        rviz::FloatPropertyWPtr effort_property_;
        rviz::FloatPropertyWPtr max_effort_property_;
    };

    JointInfo::JointInfo()
        : enabled_(true)
    {
    }

    JointInfo* EffortDisplay::getJointInfo( const std::string& joint)
    {
        M_JointInfo::iterator it = joints_.find( joint );
        if ( it == joints_.end() )
            {
                return NULL;
            }

        return it->second;
    }

    void EffortDisplay::setJointEnabled(JointInfo* joint, bool enabled)
    {
        joint->enabled_ = enabled;
        propertyChanged(joint->enabled_property_);
    }

    JointInfo* EffortDisplay::createJoint(const std::string &joint)
    {
        JointInfo *info = new JointInfo;
        joints_.insert( std::make_pair( joint, info ) );

        info->name_ = joint;
        info->enabled_ = true;
        info->effort_ = 0;
        info->max_effort_ = 0;
        info->last_update_ = ros::Time::now();

        std::string prefix = "Joints.";
        info->category_ = property_manager_->createCategory( info->name_, "", joints_category_, this);

        prefix += info->name_ + ".";

        info->enabled_property_ = property_manager_->createProperty<rviz::BoolProperty>( "Enabled", prefix, boost::bind( &JointInfo::isEnabled, info), boost::bind(&EffortDisplay::setJointEnabled, this, info, _1), info->category_, this);
        setPropertyHelpText(info->enabled_property_, "Enable or disable this individual joint.");

        info->effort_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Effort", prefix, boost::bind( &JointInfo::getEffort, info), boost::bind( &JointInfo::setEffort, info, _1), info->category_, this);
        setPropertyHelpText(info->effort_property_, "Effort value of this joint.");

        info->max_effort_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Max Effort", prefix, boost::bind( &JointInfo::getMaxEffort, info), boost::bind( &JointInfo::setMaxEffort, info, _1), info->category_, this);
        setPropertyHelpText(info->max_effort_property_, "Max Effort value of this joint.");
        updateJoint(info);

        return info;
    }

    void EffortDisplay::updateJoint(JointInfo* joint)
    {
    }

    void EffortDisplay::deleteJoint(JointInfo* joint, bool delete_properties)
    {
    }

    EffortDisplay::EffortDisplay()
	: Display()
	, scene_node_( NULL )
	, messages_received_( 0 )
	, alpha_( 1.0 )              // Default alpha is completely opaque.
	, width_( 0.02 )              // Default width
	, scale_( 1.0 )              // Default scale
        , all_enabled_( true )
    {
    }

    void EffortDisplay::onInitialize()
    {
	// Make an Ogre::SceneNode to contain all our visuals.
	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	// Set the default history length and resize the ``visuals_`` array.
	setHistoryLength( 1 );

	// A tf::MessageFilter listens to ROS messages and calls our
	// callback with them when they can be matched up with valid tf
	// transform data.
	tf_filter_ =
	    //new tf::MessageFilter<sensor_msgs::JointState>( *vis_manager_->getTFClient(),
	    new tf::MessageFilterJointState( *vis_manager_->getTFClient(),
					     "", 100, update_nh_, ros::Duration(0.1) );
	tf_filter_->connectInput( sub_ );
	tf_filter_->registerCallback( boost::bind( &EffortDisplay::incomingMessage,
						   this, _1 ));

	// FrameManager has some built-in functions to set the status of a
	// Display based on callbacks from a tf::MessageFilter.  These work
	// fine for this simple display.
	vis_manager_->getFrameManager()
	    ->registerFilterForTransformStatusCheck( tf_filter_, this );

    }

    EffortDisplay::~EffortDisplay()
    {
	unsubscribe();
	clear();
	visuals_.clear();

	delete tf_filter_;
    }

    // Clear the visuals by deleting their objects.
    void EffortDisplay::clear()
    {
	for( size_t i = 0; i < visuals_.size(); i++ ) {
	    delete visuals_[ i ];
	    visuals_[ i ] = NULL;
	}
	tf_filter_->clear();
	messages_received_ = 0;
	setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
    }

    void EffortDisplay::setTopic( const std::string& topic )
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

    void EffortDisplay::setAlpha( float alpha )
    {
	alpha_ = alpha;

	propertyChanged( alpha_property_ );
	updateColorAndAlpha();
	causeRender();
    }

    void EffortDisplay::setWidth( float width )
    {
	width_ = width;

	propertyChanged( width_property_ );
	updateColorAndAlpha();
	causeRender();
    }

    void EffortDisplay::setScale( float scale )
    {
	scale_ = scale;

	propertyChanged( scale_property_ );
	updateColorAndAlpha();
	causeRender();
    }

    // Set the current color and alpha values for each visual.
    void EffortDisplay::updateColorAndAlpha()
    {
	BOOST_FOREACH(MapEffortVisual::value_type visual, visuals_)
	{
            if ( visual ) {
                visual->setWidth( width_ );
                visual->setScale( scale_ );
            }
	}
    }

    // Set the number of past visuals to show.
    void EffortDisplay::setHistoryLength( int length )
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
	std::vector<EffortVisual*> new_visuals( history_length_, (EffortVisual*)0 );

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

    void EffortDisplay::load()
    {
	// get robot_description
	std::string urdf_string;
	update_nh_.getParam(description_param_, urdf_string);
	urdfModel = boost::shared_ptr<urdf::Model>(new urdf::Model());
	if (!urdfModel->initString(urdf_string))
	{
	    ROS_ERROR("Unable to parse URDF description!");
            setStatus(rviz::status_levels::Error, "URDF", "Unable to parse robot model description!");
	    return;
	}
        setStatus(rviz::status_levels::Ok, "URDF", "Ok");

	for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdfModel->joints_.begin(); it != urdfModel->joints_.end(); it ++ ) {
            boost::shared_ptr<urdf::Joint> joint = it->second;
	    if ( joint->type == urdf::Joint::REVOLUTE ) {
                std::string joint_name = it->first;
		boost::shared_ptr<urdf::JointLimits> limit = joint->limits;
                joints_[joint_name] = createJoint(joint_name);
                joints_[joint_name]->setMaxEffort(limit->effort);
            }
        }
    }

    void EffortDisplay::setRobotDescription( const std::string& description_param )
    {
        description_param_ = description_param;

        propertyChanged(robot_description_property_);

        if ( isEnabled() )
            {
                load();
                unsubscribe();
                subscribe();
                causeRender();
            }
    }

    void EffortDisplay::setAllEnabled(bool enabled)
    {
        all_enabled_ = enabled;

        M_JointInfo::iterator it = joints_.begin();
        M_JointInfo::iterator end = joints_.end();
        for (; it != end; ++it)
            {
                JointInfo* joint = it->second;

                setJointEnabled(joint, enabled);
            }

        propertyChanged(all_enabled_property_);
    }


    void EffortDisplay::subscribe()
    {
	// If we are not actually enabled, don't do it.
	if ( !isEnabled() )
	{
	    return;
	}

        // if urdf model is not loaded, return
        if ( ! urdfModel ) {
	    setStatus( rviz::status_levels::Warn, "URDF", "Valid robot model is not loaded" );
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

    void EffortDisplay::unsubscribe()
    {
	sub_.unsubscribe();
    }

    void EffortDisplay::onEnable()
    {
	subscribe();
    }

    void EffortDisplay::onDisable()
    {
	unsubscribe();
	clear();
    }

    // When the "Fixed Frame" changes, we need to update our
    // tf::MessageFilter and erase existing visuals.
    void EffortDisplay::fixedFrameChanged()
    {
	tf_filter_->setTargetFrame( fixed_frame_ );
	clear();
    }

    // This is our callback to handle an incoming message.
    void EffortDisplay::incomingMessage( const sensor_msgs::JointState::ConstPtr& msg )
    {
	++messages_received_;

	// Each display can have multiple status lines.  This one is called
	// "Topic" and says how many messages have been received in this case.
	std::stringstream ss;
	ss << messages_received_ << " messages received";
	setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

	// We are keeping a circular buffer of visual pointers.  This gets
	// the next one, or creates and stores it if it was missing.
	EffortVisual* visual = visuals_[ messages_received_ % history_length_ ];
	if( visual == NULL )
	  {
            visual = new EffortVisual( vis_manager_->getSceneManager(), scene_node_ , urdfModel );
	    visuals_[ messages_received_ % history_length_ ] = visual;
	  }

        V_string joints;
        int joint_num = msg->name.size();
        for (int i = 0; i < joint_num; i++ )
        {
            std::string joint_name = msg->name[i];
            JointInfo* joint_info = getJointInfo(joint_name);
            if ( !joint_info ) continue; // skip joints..

            // set effort
            joint_info->setEffort(msg->effort[i]);

            // update effort property
            if ( ros::Time::now() - joint_info->last_update_ > ros::Duration(0.2) ) {
                propertyChanged(joint_info->effort_property_);
                joint_info->last_update_ = ros::Time::now();
            }

	    const urdf::Joint* joint = urdfModel->getJoint(joint_name).get();
	    int joint_type = joint->type;
	    if ( joint_type == urdf::Joint::REVOLUTE )
	    {
		// we expects that parent_link_name equals to frame_id.
		std::string parent_link_name = joint->child_link_name;
		Ogre::Quaternion orientation;
		Ogre::Vector3 position;

		// Here we call the rviz::FrameManager to get the transform from the
		// fixed frame to the frame in the header of this Effort message.  If
		// it fails, we can't do anything else so we return.
		if( !vis_manager_->getFrameManager()->getTransform( parent_link_name,
								    ros::Time(),
								    //msg->header.stamp, // ???
								    position, orientation ))
		{
		    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
			       parent_link_name.c_str(), fixed_frame_.c_str() );
		    continue;
		}
;
		tf::Vector3 axis_joint(joint->axis.x, joint->axis.y, joint->axis.z);
		tf::Vector3 axis_z(0,0,1);
		tf::Quaternion axis_rotation(tf::tfCross(axis_joint, axis_z), tf::tfAngle(axis_joint, axis_z));
		if ( std::isnan(axis_rotation.x()) ||
		     std::isnan(axis_rotation.y()) ||
		     std::isnan(axis_rotation.z()) ) axis_rotation = tf::Quaternion::getIdentity();

		tf::Quaternion axis_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
		tf::Quaternion axis_rot = axis_orientation * axis_rotation;
		Ogre::Quaternion joint_orientation(Ogre::Real(axis_rot.w()), Ogre::Real(axis_rot.x()), Ogre::Real(axis_rot.y()), Ogre::Real(axis_rot.z()));
		visual->setFramePosition( joint_name, position );
		visual->setFrameOrientation( joint_name, joint_orientation );
                visual->setFrameEnabled( joint_name, joint_info->isEnabled() );
	    }
	}


	// Now set or update the contents of the chosen visual.
        visual->setWidth( width_ );
        visual->setScale( scale_ );
	visual->setMessage( msg );
    }

    // Override rviz::Display's reset() function to add a call to clear().
    void EffortDisplay::reset()
    {
	Display::reset();
	clear();
    }

    void EffortDisplay::createProperties()
    {
	topic_property_ =
	    property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
									     property_prefix_,
									     boost::bind( &EffortDisplay::getTopic, this ),
									     boost::bind( &EffortDisplay::setTopic, this, _1 ),
									     parent_category_,
									     this );
	setPropertyHelpText( topic_property_, "sensor_msgs::Effort topic to subscribe to." );
	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType( ros::message_traits::datatype<sensor_msgs::JointState>() );

	alpha_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Alpha",
								    property_prefix_,
								    boost::bind( &EffortDisplay::getAlpha, this ),
								    boost::bind( &EffortDisplay::setAlpha, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( alpha_property_, "0 is fully transparent, 1.0 is fully opaque." );

	width_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Width",
								    property_prefix_,
								    boost::bind( &EffortDisplay::getWidth, this ),
								    boost::bind( &EffortDisplay::setWidth, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( width_property_, "Width to drow effort circle" );

	scale_property_ =
	    property_manager_->createProperty<rviz::FloatProperty>( "Scale",
								    property_prefix_,
								    boost::bind( &EffortDisplay::getScale, this ),
								    boost::bind( &EffortDisplay::setScale, this, _1 ),
								    parent_category_,
								    this );
	setPropertyHelpText( scale_property_, "Scale to drow effort circle" );

	history_length_property_ =
	    property_manager_->createProperty<rviz::IntProperty>( "History Length",
								  property_prefix_,
								  boost::bind( &EffortDisplay::getHistoryLength, this ),
								  boost::bind( &EffortDisplay::setHistoryLength, this, _1 ),
								  parent_category_,
							    this );
	setPropertyHelpText( history_length_property_, "Number of prior measurements to display." );

        robot_description_property_ = property_manager_->createProperty<rviz::StringProperty>( "Robot Description", property_prefix_, boost::bind( &EffortDisplay::getRobotDescription, this ), boost::bind( &EffortDisplay::setRobotDescription, this, _1 ), parent_category_, this );
        rviz::setPropertyHelpText(robot_description_property_, "Name of the parameter to search for to load the robot description.");


        joints_category_ =
	    property_manager_->createCategory( "Joints",
                                               property_prefix_,
                                               parent_category_,
                                               this );
	setPropertyHelpText( joints_category_, "The list of all joints." );
        rviz::CategoryPropertyPtr cat_prop = joints_category_.lock();
        cat_prop->collapse();
        all_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty>( "All Enabled", property_prefix_, boost::bind( &EffortDisplay::getAllEnabled, this ),
                                                                                       boost::bind( &EffortDisplay::setAllEnabled, this, _1 ), joints_category_, this );
        setPropertyHelpText(all_enabled_property_, "Whether all the joints should be enabled or not.");

    }
} // end namespace jsk_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( jsk_rviz_plugin, Effort, jsk_rviz_plugin::EffortDisplay, rviz::Display )


