#ifndef JSK_RVIZ_PLUGINGS_STRING_UTIL_H_
#define JSK_RVIZ_PLUGINGS_STRING_UTIL_H_

#include <string>

namespace rviz_string
{
template <typename... Args>
static std::string format(const std::string & fmt, Args... args)
{
#pragma GCC diagnostic ignored "-Wformat-security"
  size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args...);
  std::vector<char> buf(len + 1);
  std::snprintf(&buf[0], len + 1, fmt.c_str(), args...);
  return std::string(&buf[0], &buf[0] + len);
#pragma GCC diagnostic warning "-Wformat-security"
}

static std::string replace_str(std::string str, std::string find, std::string rep)
{
  std::string::size_type pos(str.find(find));

  while (pos != std::string::npos) {
    str.replace(pos, find.length(), rep);
    pos = str.find(find, pos + rep.length());
  }

  return str;
}
}  // namespace rviz_string

#endif  // JSK_RVIZ_PLUGINGS_STRING_UTIL_H_