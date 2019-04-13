#include "image_transport_hints_property.h"

namespace jsk_rviz_plugins {

ImageTransportHintsProperty::ImageTransportHintsProperty(const char* name,
                                                         const char* description,
                                                         rviz::Property* parent,
                                                         const char* changed_slot)
    : rviz::EditableEnumProperty(name, "raw", description, parent, changed_slot) {
  addOptionStd("raw");
  addOptionStd("compressed");
  addOptionStd("theora");
}

ImageTransportHintsProperty::~ImageTransportHintsProperty() {}

image_transport::TransportHints ImageTransportHintsProperty::getTransportHints() {
  return image_transport::TransportHints(getStdString());
}

}
