//
// @file introspection.h
// @brief Lightweight helpers using rosidl_typesupport_introspection_cpp.
//
#ifndef VIO_BRIDGE__INTROSPECTION_H_
#define VIO_BRIDGE__INTROSPECTION_H_

#include <string>
#include <string_view>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace vio_bridge::introspection {

// Try to read a string field from a ROS 2 message using introspection.
// Returns true on success and assigns 'out'.
bool try_read_string_field(const void* msg,
                           const rosidl_message_type_support_t* ts,
                           std::string_view field_name,
                           std::string& out);

}  // namespace vio_bridge::introspection

#endif  // VIO_BRIDGE__INTROSPECTION_H_

