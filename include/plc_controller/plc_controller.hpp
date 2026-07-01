// Copyright 2026 CNR-STIIMA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file plc_controller.hpp
 * @brief ROS 2 Control plugin for safety PLC GPIO command/state bridging.
 * 
 * This header defines the PLCController class, a ros2_control ControllerInterface
 * plugin that bridges ROS 2 software with a PLC connected via EtherCAT. 
 * The controller implements bidirectional GPIO communication:
 * 
 * Command Flow (ROS → PLC):
 *     PlcController ROS message
 *         ↓ (plc_command_subscriber receives PlcController)
 *         ↓ (rt_command_ptr buffer stores latest command)
 *         ↓ (update() calls update_plc_commands())
 *         ↓ (matches command interface names to indices)
 *         ↓ (sets LoanedCommandInterface values via set_value())
 *     EtherCAT Driver (updates CoM registers)
 *         ↓
 *     Safety PLC (reads GPIO outputs)
 * 
 * State Flow (PLC → ROS):
 *     Safety PLC (writes GPIO inputs)
 *         ↓
 *     EtherCAT Driver (reads CoM registers)
 *         ↓ (update() calls update_plc_states())
 *         ↓ (iterates state_interface_types_)
 *         ↓ (reads LoanedStateInterface values via get_optional<double>())
 *     PlcStates ROS message
 *         ↓ (realtime_plc_state_publisher_ publishes to /PLC_controller/plc_states)
 *         ↓
 *     ROS subscribers
 * 
 * 
 * Lifecycle:
 *     1. on_init(): Load parameters from plc_controller.yaml via ParamListener
 *     2. on_configure(): Parse YAML, create subscriber/publisher, validate interfaces
 *     3. on_activate(): Create maps linking interface names to LoanedInterface references
 *     4. update() [called at 500 Hz]: Read commands, apply to PLC, publish states
 *     5. on_deactivate(): Stop processing
 * 
 * Thread Safety:
 *     - rt_command_ptr_: RealtimeBuffer for lock-free command passing from subscription thread
 *     - realtime_plc_state_publisher_: RealtimePublisher for lock-free state publishing
 *     - No explicit locking; relies on lock-free data structures
 * 
 * Performance Characteristics:
 *     - Control loop frequency: 500 Hz (2 ms period)
 *     - Command latency: <2 ms (single update cycle)
 *     - State publish latency: <2 ms (single update cycle)
 *     - Real-time safe: All memory pre-allocated, no dynamic allocation in update()
 * 
 * @note This controller requires the EtherCAT driver plugin to be loaded by
 *       the parent controller_manager.
 * @note Parameters are loaded from plc_controller.yaml during on_configure().
 * @note All interfaces must be pre-declared in sickPLC.ros2_control.urdf.
 */

#ifndef PLC_CONTROLLER__PLC_CONTROLLER_HPP_
#define PLC_CONTROLLER__PLC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "tecnobody_msgs/msg/plc_controller.hpp"
#include "tecnobody_msgs/msg/plc_states.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "plc_controller/plc_controller_parameters.hpp"

namespace plc_controller
{
/// @brief Convenience type alias for PlcController ROS message type
using CmdType = tecnobody_msgs::msg::PlcController;

/// @brief Convenience type alias for PlcStates ROS message type
using StateType = tecnobody_msgs::msg::PlcStates;

/// @brief Convenience type alias for controller lifecycle callback return type
using CallbackReturn = controller_interface::CallbackReturn;

/// @brief Convenience type alias for vector of interface name strings
using InterfacesNames = std::vector<std::string>;

/// @brief Map from interface name (string) to LoanedCommandInterface reference
using MapOfReferencesToCommandInterfaces = std::unordered_map<
  std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;

/// @brief Map from interface name (string) to LoanedStateInterface reference
using MapOfReferencesToStateInterfaces =
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

/// @brief Vector of LoanedStateInterface references for state access
using StateInterfaces =
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

/**
 * @class PLCController
 * @brief ROS 2 Control plugin bridging ROS software and safety PLC hardware.
 * 
 * This class implements the ControllerInterface for ros2_control, providing
 * bidirectional GPIO communication between ROS and a safety-certified PLC.
 * 
 * The controller manages:
 *   - 8 GPIO command outputs to PLC
 *   - 8 GPIO state inputs from PLC
 *   - Subscription to PlcController ROS messages for command input
 *   - Publication of PlcStates ROS messages for state feedback
 * 
 * Key Design Decisions:
 *   1. Real-time safe: Uses lock-free RealtimeBuffer for command passing
 *   2. Parameter-driven: All GPIO names/interfaces loaded from YAML (plc_controller.yaml)
 *   3. Flexible: Supports arbitrary GPIO count and naming via parameters
 *   4. Safe: Validates interface names at configure-time, not at runtime
 * 
 * Usage in Launch File:
 *     - Loaded via controller_manager spawner:
 *       spawner PLC_controller -c /plc_controller_manager
 *     - Configuration file: plc_controller.yaml
 *     - Hardware definition: sickPLC.ros2_control.urdf (via URDF xacro includes)
 * 
 * @see https://ros-controls.github.io/control.ros.org/
 */
class PLCController : public controller_interface::ControllerInterface
{
public:
  /// @brief Default constructor
  PLCController();

  /**
   * @brief Return list of command interfaces this controller requires.
   * @return InterfaceConfiguration specifying individual command interface names
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Return list of state interfaces this controller requires.
   * @return InterfaceConfiguration specifying individual state interface names
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Initialize controller (load parameters from YAML).
   * @return CallbackReturn::SUCCESS on success, ERROR on parameter loading failure
   */
  CallbackReturn on_init() override;

  /**
   * @brief Configure controller (subscribe to commands, publish states).
   * @param previous_state Current lifecycle state before transition
   * @return CallbackReturn::SUCCESS on success, ERROR on configuration failure
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate controller (create interface maps, prepare for update loop).
   * @param previous_state Current lifecycle state before transition
   * @return CallbackReturn::SUCCESS on success, ERROR on activation failure
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate controller (stop processing, release resources).
   * @param previous_state Current lifecycle state before transition
   * @return CallbackReturn::SUCCESS on deactivation
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Main control loop (called at 500 Hz by controller_manager).
   * 
   * This is the core real-time safe callback executed at 500 Hz. It:
   *   1. Calls update_plc_states() to read state inputs and publish
   *   2. Calls update_plc_commands() to write command outputs
   * 
   * @param time Current ROS time
   * @param period Control loop period (2 ms)
   * @return return_type::OK on success, ERROR on exception
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// ========== Command/State Interface Management ==========

  /**
   * @brief Extract command interface names from YAML parameters.
   * 
   * Iterates over params_.command_interfaces and builds command_interface_types_
   * vector with fully-qualified names: "PLC_node/estop", "PLC_node/sonar_teach", etc.
   * 
   * Called during on_configure() to populate the interface list required by
   * command_interface_configuration().
   */
  void store_command_interface_types();

  /**
   * @brief Extract state interface names from YAML parameters.
   * 
   * Two modes:
   *   1. If state_interfaces empty in YAML: Use all available interfaces from URDF
   *      (calls set_all_state_interfaces_of_configured_gpios())
   *   2. If state_interfaces specified in YAML: Use only those interfaces
   * 
   * Populates state_interface_types_ with fully-qualified names.
   * Called during on_configure().
   */
  void store_state_interface_types();

  /**
   * @brief Initialize PlcStates message with interface names.
   * 
   * Pre-allocates PlcStates message structure with correct interface names
   * and empty value vector. Called during on_activate() to prepare the
   * message template for subsequent state updates.
   */
  void initialize_plc_state_msg();

  /// ========== Runtime Interface Operations ==========

  /**
   * @brief Publish current PLC state inputs to ROS subscribers.
   * 
   * This function:
   *   1. Locks realtime_plc_state_publisher_ for thread-safe access
   *   2. Iterates state_interface_types_ and reads each LoanedStateInterface
   *   3. Converts double values to uint8_t (GPIO are boolean/digital)
   *   4. Populates PlcStates message with interface names and values
   *   5. Publishes to /plc_controller_manager/PLC_controller/plc_states
   * 
   * Called every 2 ms (500 Hz) from update(). Uses try_lock() for non-blocking
   * operation; skips publish if lock unavailable (no blocking in real-time loop).
   */
  void update_plc_states();

  /**
   * @brief Apply ROS commands to PLC via command interfaces.
   * 
   * This function:
   *   1. Reads latest PlcController message from rt_command_ptr_ buffer
   *   2. Validates interface_names length matches values length
   *   3. For each command interface in command_interfaces_map_:
   *      - Finds corresponding index in command message
   *      - Calls set_value() on LoanedCommandInterface (sends to EtherCAT driver)
   *   4. Logs warnings if interface not found in command message (graceful degradation)
   * 
   * Called every 2 ms (500 Hz) from update(). Non-blocking; returns immediately
   * if no new command received.
   * 
   * @return return_type::OK on success, ERROR on validation failure
   */
  controller_interface::return_type update_plc_commands();

  /// ========== Template Utilities (Real-Time Safe) ==========

  /**
   * @brief Create map from interface names (strings) to interface references.
   * 
   * Generic template function for both command and state interfaces. Creates
   * hash map linking fully-qualified interface names to LoanedInterface references,
   * enabling O(1) lookup during update().
   * 
   * @tparam T LoanedCommandInterface or LoanedStateInterface
   * @param interfaces_from_params List of interface names from YAML
   * @param configured_interfaces Vector of LoanedInterface from controller_manager
   * @return Hash map from interface name → reference
   */
  template <typename T>
  std::unordered_map<std::string, std::reference_wrapper<T>>
  create_map_of_references_to_interfaces(
    const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces);

  /**
   * @brief Validate that configured interfaces match requested interfaces.
   * 
   * Security check: Ensures controller_manager provided exactly the interfaces
   * we requested. Logs detailed error if mismatch (indicates misconfiguration).
   * 
   * @tparam T LoanedCommandInterface or LoanedStateInterface
   * @param interfaces_from_params Interfaces requested in YAML
   * @param configured_interfaces Interfaces provided by controller_manager
   * @return true if counts match, false otherwise (logs errors)
   */
  template <typename T>
  bool check_if_configured_interfaces_matches_received(
    const InterfacesNames & interfaces_from_params, const T & configured_interfaces);

  /// ========== Parameter Parsing Utilities ==========

  /**
   * @brief Refresh dynamic parameters from YAML listener.
   * 
   * ROS 2 parameter listener pattern: calls refresh_dynamic_parameters() to
   * detect parameter changes, then reads updated values via get_params().
   * Used to support runtime parameter reconfiguration (not critical for PLC).
   * 
   * @return true on success
   */
  bool update_dynamic_map_parameters();

  /**
   * @brief Check if all state interfaces should be auto-discovered from URDF.
   * 
   * If YAML specifies empty state_interfaces list, controller auto-discovers
   * all available interfaces from URDF hardware definition instead of requiring
   * manual specification. This reduces YAML boilerplate for simple cases.
   * 
   * @return true if state_interfaces empty (auto-discovery mode)
   */
  bool should_broadcast_all_interfaces_of_configured_gpios() const;

  /**
   * @brief Set state interfaces to include all available GPIO interfaces from URDF.
   * 
   * Parses URDF hardware definition (via parse_control_resources_from_urdf()),
   * extracts all state_interface definitions for the configured GPIO, and adds
   * them to state_interface_types_ vector.
   * 
   * Used when state_interfaces not explicitly configured in YAML (auto-discovery).
   */
  InterfacesNames set_all_state_interfaces_of_configured_gpios();

  /**
   * @brief Parse URDF hardware information to extract GPIO definitions.
   * 
   * Calls hardware_interface::parse_control_resources_from_urdf() to parse the
   * URDF (sickPLC.config.urdf) and extract GPIO component information including
   * state/command interface names.
   * 
   * @return Vector of hardware_interface::ComponentInfo structs for all GPIOs
   */
  std::vector<hardware_interface::ComponentInfo> get_gpios_from_urdf() const;

  /**
   * @brief Extract state interface names for specific GPIO from mapping.
   * 
   * Iterates state_interfaces_map_ and filters to interfaces belonging to
   * specified GPIO name. Used to populate PlcStates message with correct
   * interface names.
   * 
   * @param gpio_name GPIO identifier (e.g., "PLC_node")
   * @return Vector of interface names for this GPIO
   */
  InterfacesNames get_plc_state_interfaces_names(const std::string & gpio_name) const;

  /// ========== State Variables (Pre-allocated for Real-Time Safety) ==========

  /// @brief Fully-qualified command interface names from YAML
  /// Example: ["PLC_node/estop", "PLC_node/force_sensors_pwr", ...]
  InterfacesNames command_interface_types_;

  /// @brief Fully-qualified state interface names from YAML
  /// Example: ["PLC_node/estop", "PLC_node/reset", ...]
  InterfacesNames state_interface_types_;

  /// @brief Map: interface name (string) → LoanedCommandInterface reference
  /// Enables O(1) lookup of command interface by name during update()
  MapOfReferencesToCommandInterfaces command_interfaces_map_;

  /// @brief Map: interface name (string) → LoanedStateInterface reference
  /// Enables O(1) lookup of state interface by name during update()
  MapOfReferencesToStateInterfaces state_interfaces_map_;

  /// @brief Lock-free buffer for ROS command message (subscription thread → update thread)
  /// Subscriber writes latest PlcController message to buffer without blocking real-time thread
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_{};

  /// @brief ROS subscription to PlcController command topic
  /// Topic: /plc_controller_manager/PLC_controller/plc_commands
  /// QoS: Best-effort (fire-and-forget for low-latency updates)
  rclcpp::Subscription<CmdType>::SharedPtr plc_command_subscriber_{};

  /// @brief ROS publisher for PlcStates state feedback
  /// Topic: /plc_controller_manager/PLC_controller/plc_states
  /// QoS: System default (reliable, keep-last=1)
  std::shared_ptr<rclcpp::Publisher<StateType>> plc_state_publisher_{};

  /// @brief Real-time safe publisher for PlcStates (lock-free)
  /// Wraps plc_state_publisher_ to enable publication from real-time thread
  /// without locks or dynamic allocation
  std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_plc_state_publisher_{};

  /// @brief ROS 2 parameter listener (auto-generated from parameter_library)
  /// Listens for parameter changes and provides access to current values
  std::shared_ptr<plc_controller_parameters::ParamListener> param_listener_{};

  /// @brief Current parameter values loaded from plc_controller.yaml
  /// Includes gpios list, command_interfaces map, state_interfaces map
  plc_controller_parameters::Params params_;
};

}  // namespace plc_controller

#endif  // PLC_CONTROLLER__PLC_CONTROLLER_HPP_