// Copyright 2026 CNR-STIIMA
//
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

TEST(PlcControllerPlugin, LoadsControllerInterfacePlugin)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");

  auto controller = loader.createSharedInstance("plc_controller/PLCController");
  ASSERT_NE(controller, nullptr);

  const auto command_config = controller->command_interface_configuration();
  EXPECT_EQ(
    command_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_TRUE(command_config.names.empty());

  const auto state_config = controller->state_interface_configuration();
  EXPECT_EQ(
    state_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_TRUE(state_config.names.empty());
}
