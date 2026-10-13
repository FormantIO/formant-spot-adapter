#include <iostream>
#include <string>
#include <vector>

#include "formant_spot_adapter/command_params.hpp"

namespace {

int failures = 0;

void expect_eq(const std::string& actual, const std::string& expected, const char* what) {
  if (actual == expected) return;
  ++failures;
  std::cerr << "command_params_test failed: " << what << ": got \"" << actual
            << "\", want \"" << expected << "\"" << std::endl;
}

const std::vector<std::string> kNameKeys = {"name", "waypoint", "waypoint_name", "alias"};
const std::vector<std::string> kMapIdKeys = {"map_id", "map", "id"};

void test_missing_key_is_empty() {
  using fsa::ExtractCommandParam;
  expect_eq(ExtractCommandParam("name=kitchen", {"map_uuid"}), "",
            "a key that is absent must not return the whole text");
  expect_eq(ExtractCommandParam("kitchen", {"map_uuid"}), "",
            "a bare value is not a value for a named key");
  expect_eq(ExtractCommandParam("kitchen", {"waypoint_id"}), "",
            "a bare value is not a waypoint id");
  expect_eq(ExtractCommandParam("1.5", {"x", "seed_x"}), "",
            "a bare number is not an x coordinate");
  expect_eq(ExtractCommandParam("", {"name"}), "", "empty text has no parameters");
  expect_eq(ExtractCommandParam("   ", {"name"}), "", "blank text has no parameters");
}

void test_key_value_pairs() {
  using fsa::ExtractCommandParam;
  const std::string pose = "map_uuid=abc-123, x=1.5, y=-2, yaw_deg=90";
  expect_eq(ExtractCommandParam(pose, {"map_uuid"}), "abc-123", "map_uuid from goto_pose text");
  expect_eq(ExtractCommandParam(pose, {"x", "seed_x"}), "1.5", "x from goto_pose text");
  expect_eq(ExtractCommandParam(pose, {"y", "seed_y"}), "-2", "y from goto_pose text");
  expect_eq(ExtractCommandParam(pose, {"yaw_deg", "heading_deg", "degrees"}), "90",
            "yaw_deg from goto_pose text");
  expect_eq(ExtractCommandParam(pose, {"yaw_rad", "yaw", "theta", "heading_rad"}), "",
            "yaw_rad is absent from goto_pose text");
  expect_eq(ExtractCommandParam("seed_x=3;seed_y=4", {"x", "seed_x"}), "3",
            "second key in the list, semicolon separated");
  expect_eq(ExtractCommandParam("seed_x=3\nseed_y=4", {"y", "seed_y"}), "4",
            "newline separated");
  expect_eq(ExtractCommandParam("Name= Dock Entry ", {"name"}), "Dock Entry",
            "keys are case-insensitive and values are trimmed");
  expect_eq(ExtractCommandParam("waypoint_name=dock", kNameKeys), "dock",
            "a later key in the list still matches");
}

void test_keys_match_whole_words_only() {
  using fsa::ExtractCommandParam;
  expect_eq(ExtractCommandParam("waypoint_id=wp-1", kMapIdKeys), "",
            "id must not match inside waypoint_id");
  expect_eq(ExtractCommandParam("map_uuid=abc", kMapIdKeys), "",
            "id must not match inside map_uuid");
  expect_eq(ExtractCommandParam("yaw_deg=90", {"degrees", "deg"}), "",
            "deg must not match inside yaw_deg");
  expect_eq(ExtractCommandParam("seed_x=3", {"x"}), "", "x must not match inside seed_x");
  expect_eq(ExtractCommandParam("map_uuid=abc, id=7", kMapIdKeys), "7",
            "a whole-word match later in the text is still found");
}

void test_json() {
  using fsa::ExtractCommandParam;
  expect_eq(ExtractCommandParam("{\"map_id\":\"warehouse_a\"}", kMapIdKeys), "warehouse_a",
            "quoted JSON value");
  expect_eq(ExtractCommandParam("{\"x\": 1.25, \"y\": 2}", {"x", "seed_x"}), "1.25",
            "unquoted JSON number");
  expect_eq(ExtractCommandParam("{\"x\": 1.25, \"y\": 2}", {"y", "seed_y"}), "2",
            "last JSON number");
  expect_eq(ExtractCommandParam("{\"map_id\":\"warehouse_a\"}", {"waypoint_id"}), "",
            "JSON without the key");
}

void test_ui_command_strings() {
  using fsa::ExtractCommandParam;
  using fsa::ExtractCommandParamOrBareValue;
  // The exact shapes ui/map-navigation builds.
  const std::string by_id = "map_uuid=6f1c-22, waypoint_id=wp-9a";
  expect_eq(ExtractCommandParam(by_id, {"map_uuid"}), "6f1c-22", "UI goto by id: map_uuid");
  expect_eq(ExtractCommandParam(by_id, {"waypoint_id"}), "wp-9a", "UI goto by id: waypoint_id");
  expect_eq(ExtractCommandParamOrBareValue(by_id, kNameKeys), "",
            "UI goto by id carries no name");
  const std::string by_name = "map_uuid=6f1c-22, name=charging dock";
  expect_eq(ExtractCommandParam(by_name, {"waypoint_id"}), "", "UI goto by name: no waypoint_id");
  expect_eq(ExtractCommandParamOrBareValue(by_name, kNameKeys), "charging dock",
            "UI goto by name: name");
}

void test_edge_shapes() {
  using fsa::ExtractCommandParam;
  using fsa::ExtractCommandParamOrBareValue;
  expect_eq(ExtractCommandParam("name = kitchen", {"name"}), "kitchen",
            "spaces around = are allowed");
  expect_eq(ExtractCommandParam("name=", {"name"}), "", "an empty value is empty");
  expect_eq(ExtractCommandParam("name=a=b", {"name"}), "a=b", "a value may contain =");
  expect_eq(ExtractCommandParamOrBareValue("a=b", kNameKeys), "",
            "an unknown key=value is not a bare value");
  expect_eq(ExtractCommandParamOrBareValue("map_name=site_a", {"name", "map_id", "map", "id"}),
            "", "name must not match inside map_name");
  expect_eq(ExtractCommandParamOrBareValue("{\"name\":\"kitchen\"}", kNameKeys), "kitchen",
            "JSON through the bare-value lookup");
  // Known limitation, pinned: a key is still found inside an earlier value.
  expect_eq(ExtractCommandParam("name=dock entry, id=7", kMapIdKeys), "7",
            "key=value later in the text is found");
}

void test_bare_value_fallback() {
  using fsa::ExtractCommandParamOrBareValue;
  expect_eq(ExtractCommandParamOrBareValue("kitchen", kNameKeys), "kitchen",
            "a bare value is the positional parameter");
  expect_eq(ExtractCommandParamOrBareValue("  dock entry \n", kNameKeys), "dock entry",
            "a bare value is trimmed");
  expect_eq(ExtractCommandParamOrBareValue("90", {"degrees", "deg"}), "90",
            "a bare number is the positional parameter");
  expect_eq(ExtractCommandParamOrBareValue("name=kitchen", kNameKeys), "kitchen",
            "a keyed value still wins");
  expect_eq(ExtractCommandParamOrBareValue("waypoint_id=wp-1", kNameKeys), "",
            "another parameter's key=value is not a name");
  expect_eq(ExtractCommandParamOrBareValue("map_uuid=abc", kMapIdKeys), "",
            "another parameter's key=value is not a map id");
  expect_eq(ExtractCommandParamOrBareValue("{\"other\": 1}", kNameKeys), "",
            "JSON without the key is not a bare value");
  expect_eq(ExtractCommandParamOrBareValue("", kNameKeys), "", "empty text has no bare value");
}

}  // namespace

int main() {
  test_missing_key_is_empty();
  test_key_value_pairs();
  test_keys_match_whole_words_only();
  test_json();
  test_ui_command_strings();
  test_edge_shapes();
  test_bare_value_fallback();

  if (failures > 0) {
    std::cerr << "command_params_test: " << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "command_params_test: all checks passed" << std::endl;
  return 0;
}
