#include "formant_spot_adapter/command_params.hpp"

#include <algorithm>
#include <cctype>

namespace fsa {

namespace {

std::string trim(const std::string& in) {
  size_t begin = 0;
  while (begin < in.size() && std::isspace(static_cast<unsigned char>(in[begin]))) ++begin;
  size_t end = in.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(in[end - 1]))) --end;
  return in.substr(begin, end - begin);
}

std::string to_lower(const std::string& in) {
  std::string out = in;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

bool is_key_char(char c) {
  return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

// Position just past `key=` where `key` is a whole word, or npos. Spaces between
// the key and `=` are allowed.
size_t find_key_value_start(const std::string& lowered, const std::string& key) {
  const std::string lowered_key = to_lower(key);
  if (lowered_key.empty()) return std::string::npos;
  size_t pos = lowered.find(lowered_key);
  while (pos != std::string::npos) {
    size_t after = pos + lowered_key.size();
    while (after < lowered.size() && (lowered[after] == ' ' || lowered[after] == '\t')) ++after;
    const bool starts_word = pos == 0 || !is_key_char(lowered[pos - 1]);
    if (starts_word && after < lowered.size() && lowered[after] == '=') return after + 1;
    pos = lowered.find(lowered_key, pos + 1);
  }
  return std::string::npos;
}

bool extract(const std::string& trimmed, const std::vector<std::string>& keys,
             std::string* out) {
  const std::string lowered = to_lower(trimmed);
  for (const auto& key : keys) {
    const std::string quoted_key = "\"" + key + "\"";
    size_t pos = trimmed.find(quoted_key);
    if (pos != std::string::npos) {
      pos = trimmed.find(':', pos + quoted_key.size());
      if (pos != std::string::npos) {
        ++pos;
        while (pos < trimmed.size() && std::isspace(static_cast<unsigned char>(trimmed[pos]))) ++pos;
        if (pos < trimmed.size() && trimmed[pos] == '"') {
          ++pos;
          size_t end = trimmed.find('"', pos);
          if (end != std::string::npos) {
            *out = trim(trimmed.substr(pos, end - pos));
            return true;
          }
        } else {
          size_t end = pos;
          while (end < trimmed.size() && trimmed[end] != ',' && trimmed[end] != '}') ++end;
          *out = trim(trimmed.substr(pos, end - pos));
          return true;
        }
      }
    }

    pos = find_key_value_start(lowered, key);
    if (pos != std::string::npos) {
      size_t end = trimmed.find_first_of(",;\n\r", pos);
      if (end == std::string::npos) end = trimmed.size();
      *out = trim(trimmed.substr(pos, end - pos));
      return true;
    }
  }
  return false;
}

}  // namespace

std::string ExtractCommandParam(const std::string& input,
                                const std::vector<std::string>& keys) {
  std::string value;
  if (extract(trim(input), keys, &value)) return value;
  return "";
}

std::string ExtractCommandParamOrBareValue(const std::string& input,
                                           const std::vector<std::string>& keys) {
  const std::string trimmed = trim(input);
  std::string value;
  if (extract(trimmed, keys, &value)) return value;
  const bool bare = !trimmed.empty() && trimmed.front() != '{' &&
                    trimmed.find('=') == std::string::npos;
  return bare ? trimmed : "";
}

}  // namespace fsa
