#include "vio_bridge/introspection.h"

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace vio_bridge::introspection {

using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::MessageMember;

static inline const MessageMembers* as_members(const rosidl_message_type_support_t* ts)
{
  return ts ? reinterpret_cast<const MessageMembers*>(ts->data) : nullptr;
}

bool try_read_string_field(const void* msg,
                           const rosidl_message_type_support_t* ts,
                           std::string_view field_name,
                           std::string& out)
{
  if (!msg || !ts) return false;
  const MessageMembers* members = as_members(ts);
  if (!members) return false;
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const MessageMember& m = members->members_[i];
    if (m.name_ == field_name) {
      const auto* sptr = reinterpret_cast<const std::string*>(static_cast<const uint8_t*>(msg) + m.offset_);
      if (sptr) { out = *sptr; return true; }
      return false;
    }
  }
  return false;
}

}  // namespace vio_bridge::introspection
