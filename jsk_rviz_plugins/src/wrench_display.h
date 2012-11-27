#ifndef WRENCHSTAMPED_DISPLAY_H
#define WRENCHSTAMPED_DISPLAY_H

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rviz/display.h>

namespace Ogre
{
    class SceneNode;
}

namespace jsk_rviz_plugin
{

    class EfforVisual;

    class WrenchStampedDisplay: public rviz::Display
    {
    public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	WrenchStampedDisplay();
	virtual ~WrenchStampedDisplay();

	// Overrides of public virtual functions from the Display class.
	virtual void onInitialize();
	virtual void fixedFrameChanged();
	virtual void reset();
	virtual void createProperties();

	// Setter and getter functions for user-editable properties.
	void setTopic(const std::string& topic);
	const std::string& getTopic() { return topic_; }

	void setForceColor( const rviz::Color& color );
	void setTorqueColor( const rviz::Color& color );
	const rviz::Color& getForceColor() { return force_color_; }
	const rviz::Color& getTorqueColor() { return torque_color_; }

	void setAlpha( float alpha );
	float getAlpha() { return alpha_; }

	void setHistoryLength( int history_length );
	int getHistoryLength() const { return history_length_; }

	void setScale( float scale );
	float getScale() { return scale_; }

	void setWidth( float width );
	float getWidth() { return width_; }

	// Overrides of protected virtual functions from Display.  As much
	// as possible, when Displays are not enabled, they should not be
	// subscribed to incoming data and should not show anything in the
	// 3D view.  These functions are where these connections are made
	// and broken.
    protected:
	virtual void onEnable();
	virtual void onDisable();

	// Function to handle an incoming ROS message.
    private:
	void incomingMessage( const geometry_msgs::WrenchStamped::ConstPtr& msg );

	// Internal helpers which do the work of subscribing and
	// unsubscribing from the ROS topic.
	void subscribe();
	void unsubscribe();

	// A helper to clear this display back to the initial state.
	void clear();

	// Helper function to apply color and alpha to all visuals.
        void updateVisual();

	// Storage for the list of visuals par each joint intem
	typedef std::vector<WrenchStampedVisual*> MapWrenchStampedVisual;
	MapWrenchStampedVisual visuals_;

	// A node in the Ogre scene tree to be the parent of all our visuals.
	Ogre::SceneNode* scene_node_;

	// Data input: Subscriber and tf message filter.
	message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_;
        tf::MessageFilter<geometry_msgs::WrenchStamped>* tf_filter_;
	int messages_received_;

	// User-editable property variables.
        rviz::Color force_color_, torque_color_;
	std::string topic_;
        float alpha_, scale_, width_;
	int history_length_;

	// Property objects for user-editable properties.
        rviz::ColorPropertyWPtr force_color_property_, torque_color_property_;
	rviz::ROSTopicStringPropertyWPtr topic_property_;
        rviz::FloatPropertyWPtr alpha_property_, scale_property_, width_property_;
	rviz::IntPropertyWPtr history_length_property_;
    };
} // end namespace rviz_plugin_tutorials

#endif // WRENCHSTAMPED_DISPLAY_H
