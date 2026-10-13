#pragma once

#include <string>
#include <vector>

namespace fsa {

// Parameters arrive as the free text of a Formant command, either as
// `key=value` pairs separated by `,` `;` or newlines, or as a JSON object.

// Returns the value of the first of `keys` present in `input`, or an empty
// string when none of them is. A key only matches as a whole word, so `id` does
// not match inside `map_uuid=...`.
std::string ExtractCommandParam(const std::string& input,
                                const std::vector<std::string>& keys);

// Same lookup, for the one parameter a command accepts positionally: when
// `input` is a bare value (no `key=value` pair and not JSON) the whole text is
// returned, so an operator can type `kitchen` instead of `name=kitchen`.
std::string ExtractCommandParamOrBareValue(const std::string& input,
                                           const std::vector<std::string>& keys);

}  // namespace fsa
