#ifndef JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <string>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/property.hpp>
#else
#include <rviz/properties/property.h>
#include <rviz/properties/editable_enum_property.h>
#include <image_transport/transport_hints.h>
#endif

namespace jsk_rviz_plugins {

class ImageTransportHintsProperty : public rviz::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz::Property* parent, const char* changed_slot);
  ~ImageTransportHintsProperty();

#if ROS_VERSION_MAJOR != 1
  // ROS 2 image_transport takes the transport as a plain string, there is
  // no image_transport::TransportHints to hand out.
  std::string getTransport();
#else
  image_transport::TransportHints getTransportHints();
#endif
};
}
#endif
