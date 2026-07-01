// Copyright 2026 CNR-STIIMA
// SPDX-License-Identifier: Apache-2.0

#ifndef PLC_CONTROLLER__PARAMETER_UTILS_HPP_
#define PLC_CONTROLLER__PARAMETER_UTILS_HPP_

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "plc_controller/plc_controller.hpp"

namespace plc_controller
{
namespace parameter_utils
{
/**
 * @brief Flatten a mapping of GPIO names to interface lists into fully qualified names.
 *
 * @tparam GpioMap Type of the GPIO map (must provide `.interfaces` with a string vector).
 * @param gpios_map Map from gpio_name -> interface container
 * @return InterfacesNames Vector of "gpio/interface" strings in iteration order.
 */
template <typename GpioMap>
InterfacesNames flatten_gpio_interface_map(const GpioMap & gpios_map)
{
  InterfacesNames result;
  for (const auto & [gpio_name, interface_data] : gpios_map)
  {
    std::transform(
      interface_data.interfaces.cbegin(), interface_data.interfaces.cend(),
      std::back_inserter(result),
      [&gpio_name](const std::string & interface_name) { return gpio_name + '/' + interface_name; });
  }
  return result;
}

/**
 * @brief Check whether all GPIO entries expose an empty interface list.
 *
 * @tparam GpioMap Type of the GPIO map (must provide `.interfaces` with a string vector).
 * @param gpios_map Map from gpio_name -> interface container
 * @return true if every entry has an empty interface list
 * @return false otherwise
 */
template <typename GpioMap>
bool all_interface_lists_empty(const GpioMap & gpios_map)
{
  return std::all_of(
    gpios_map.cbegin(), gpios_map.cend(),
    [](const auto & gpio_entry) { return gpio_entry.second.interfaces.empty(); });
}

}  // namespace parameter_utils
}  // namespace plc_controller

#endif  // PLC_CONTROLLER__PARAMETER_UTILS_HPP_
