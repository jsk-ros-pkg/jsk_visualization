#ifndef JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#ifndef Q_MOC_RUN
#include <string>

#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/property.hpp>
#endif

namespace jsk_rviz_plugins {

class ImageTransportHintsProperty : public rviz_common::properties::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz_common::properties::Property* parent,
                              const char* changed_slot);
  ~ImageTransportHintsProperty();

  // In ROS 2 image_transport takes the transport as a plain string, so there is
  // no image_transport::TransportHints equivalent to hand out here.
  std::string getTransport();
};
}
#endif
