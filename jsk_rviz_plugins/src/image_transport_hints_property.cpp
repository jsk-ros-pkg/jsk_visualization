#include "image_transport_hints_property.h"

namespace jsk_rviz_plugins {

ImageTransportHintsProperty::ImageTransportHintsProperty(
    const char* name, const char* description,
    rviz_common::properties::Property* parent, const char* changed_slot)
    : rviz_common::properties::EditableEnumProperty(name, "raw", description, parent,
                                                    changed_slot) {
  addOptionStd("raw");
  addOptionStd("compressed");
  addOptionStd("theora");
}

ImageTransportHintsProperty::~ImageTransportHintsProperty() {}

std::string ImageTransportHintsProperty::getTransport() {
  return getStdString();
}

}
