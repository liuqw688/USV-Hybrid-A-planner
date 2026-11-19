/**
 * @file json_util.hpp
 * @brief JSON 字符串转义与数值序列化。
 */

#ifndef USV_MAP__JSON_UTIL_HPP_
#define USV_MAP__JSON_UTIL_HPP_

#include <charconv>
#include <cmath>
#include <cstdio>
#include <string>

namespace usv_map
{
namespace json
{

/** @brief 转义 JSON 字符串内容（UTF-8 非 ASCII 字符原样保留）。 */
inline std::string escape(const std::string & value)
{
  std::string out;
  out.reserve(value.size() + 8);
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
          out += buffer;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out;
}

/** @brief 带引号的 JSON 字符串。 */
inline std::string text(const std::string & value) { return "\"" + escape(value) + "\""; }

/** @brief 最短往返浮点文本；整数值补 `.0`。 */
inline std::string number(double value)
{
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "Infinity" : "-Infinity";
  }
  char buffer[40];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  std::string out(buffer, result.ptr);
  if (out.find_first_of(".eE") == std::string::npos) {
    out += ".0";
  }
  return out;
}

}  // namespace json
}  // namespace usv_map

#endif  // USV_MAP__JSON_UTIL_HPP_
