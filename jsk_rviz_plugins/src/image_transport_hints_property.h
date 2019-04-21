#ifndef JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#include <rviz/properties/property.h>
#include <rviz/properties/editable_enum_property.h>
#include <image_transport/transport_hints.h>

namespace jsk_rviz_plugins {

class ImageTransportHintsProperty : public rviz::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz::Property* parent, const char* changed_slot);
  ~ImageTransportHintsProperty();

  image_transport::TransportHints getTransportHints();
};
}
#endif
