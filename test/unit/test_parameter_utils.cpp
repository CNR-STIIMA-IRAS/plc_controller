// Copyright 2026 CNR-STIIMA
//
// SPDX-License-Identifier: Apache-2.0

#include <map>
#include <string>

#include <gtest/gtest.h>

#include "plc_controller/parameter_utils.hpp"

namespace plc_controller
{
namespace
{
struct FakeInterfaceSet
{
  std::vector<std::string> interfaces;
};
}  // namespace

TEST(ParameterUtils, FlattenGpioInterfaceMapBuildsQualifiedNames)
{
  std::map<std::string, FakeInterfaceSet> config{
    {"alpha_gpio", FakeInterfaceSet{std::vector<std::string>{"one", "two"}}},
    {"beta_gpio", FakeInterfaceSet{std::vector<std::string>{"enable"}}},
  };

  InterfacesNames flattened;
  flattened = parameter_utils::flatten_gpio_interface_map(config);

  ASSERT_EQ(flattened.size(), 3u);
  EXPECT_EQ(flattened[0], "alpha_gpio/one");
  EXPECT_EQ(flattened[1], "alpha_gpio/two");
  EXPECT_EQ(flattened[2], "beta_gpio/enable");
}

TEST(ParameterUtils, FlattenGpioInterfaceMapHandlesEmptyMap)
{
  std::map<std::string, FakeInterfaceSet> config{};
  InterfacesNames flattened;
  flattened = parameter_utils::flatten_gpio_interface_map(config);
  EXPECT_TRUE(flattened.empty());
}

TEST(ParameterUtils, AllInterfaceListsEmptyDetectsEmpty)
{
  std::map<std::string, FakeInterfaceSet> config{
    {"foo", FakeInterfaceSet{std::vector<std::string>{}}},
    {"bar", FakeInterfaceSet{std::vector<std::string>{}}},
  };

  EXPECT_TRUE(parameter_utils::all_interface_lists_empty(config));
}

TEST(ParameterUtils, AllInterfaceListsEmptyDetectsNonEmpty)
{
  std::map<std::string, FakeInterfaceSet> config{
    {"foo", FakeInterfaceSet{std::vector<std::string>{}}},
    {"bar", FakeInterfaceSet{std::vector<std::string>{"state"}}},
  };

  EXPECT_FALSE(parameter_utils::all_interface_lists_empty(config));
}

}  // namespace plc_controller
