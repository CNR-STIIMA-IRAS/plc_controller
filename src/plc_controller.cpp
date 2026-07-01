// Copyright 2026 CNR-STIIMA
//
// SPDX-License-Identifier: Apache-2.0

#include "plc_controller/plc_controller.hpp"
#include "plc_controller/parameter_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <sys/types.h>
#include <iostream>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"


namespace
{
/**
 * @brief Helper function to log interface information for debugging.
 * 
 * Used during configuration validation to print expected vs actual interfaces.
 * 
 * @tparam T: LoanedCommandInterface or LoanedStateInterface type
 * @param logger ROS logger for output
 * @param command_interfaces Map of interfaces to debug print
 */
template <typename T>
void print_interface(const rclcpp::Logger & logger, const T & command_interfaces)
{
  for (const auto & [interface_name, value] : command_interfaces)
  {
    RCLCPP_ERROR(logger, "Got %s", interface_name.c_str());
  }
}

/**
 * @brief Extract GPIO component definitions from hardware_interface array.
 * 
 * Utility to collect all GPIO components from multiple hardware definitions
 * into a single vector for easier searching.
 * 
 * @param hardware_infos Vector of HardwareInfo from URDF parsing
 * @return std::vector<ComponentInfo>: All GPIO components from all hardware blocks
 */
std::vector<hardware_interface::ComponentInfo> extract_gpios_from_hardware_info(
  const std::vector<hardware_interface::HardwareInfo> & hardware_infos)
{
  std::vector<hardware_interface::ComponentInfo> result;
  for (const auto & hardware_info : hardware_infos)
  {
    std::copy(
      hardware_info.gpios.begin(), hardware_info.gpios.end(), std::back_insert_iterator(result));
  }
  return result;
}
}  // namespace


namespace plc_controller
{

// ========== Constructor ==========

/**
 * Default constructor initializing base ControllerInterface.
 * No-op implementation as all initialization happens in on_init().
 */
PLCController::PLCController() : controller_interface::ControllerInterface() {}


// ========== Lifecycle Callbacks ==========

/**
 * Initialize controller by loading YAML parameters.
 * 
 * Lifecycle Stage: Uninitialized → Initialized
 * 
 * Actions:
 *   1. Create ParamListener connected to node parameter interface
 *   2. Load initial parameters via listener->get_params()
 *   3. Catch and log any parameter loading exceptions
 * 
 * Expected parameters from plc_controller.yaml:
 *   gpios: [list of GPIO names to control]
 *   command_interfaces: {gpio_name: {interfaces: [list of command interface names]}}
 *   state_interfaces: {gpio_name: {interfaces: [list of state interface names]}}
 * 
 * Exception Safety: Catches std::exception, logs via fprintf, returns ERROR
 * 
 * @return CallbackReturn::SUCCESS on parameter load success
 * @return CallbackReturn::ERROR on exception during parameter loading
 */
CallbackReturn PLCController::on_init()
try
{
  param_listener_ = std::make_shared<plc_controller_parameters::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}


/**
 * Configure controller for execution.
 * 
 * Lifecycle Stage: Initialized → Configured (ready for activation)
 * 
 * Actions:
 *   1. Refresh dynamic parameters from YAML listener
 *   2. Parse command interface names from params into command_interface_types_
 *   3. Parse state interface names from params into state_interface_types_
 *   4. Create ROS subscription to PlcController messages (best_effort QoS)
 *      - Callback stores incoming messages in rt_command_ptr_ for update() thread
 *   5. Create ROS publisher for PlcStates messages
 *   6. Create RealtimePublisher wrapper for lock-free state publishing
 *   7. Validate at least one command or state interface configured
 * 
 * Validation:
 *   - If no commands and no states: logs ERROR, returns CallbackReturn::ERROR
 *   - Otherwise: logs INFO "configure successful"
 * 
 * Exception Safety: Catches std::exception during any step, logs via fprintf, returns ERROR
 * 
 * @param previous_state Lifecycle state before transition (unused)
 * @return CallbackReturn::SUCCESS on successful configuration
 * @return CallbackReturn::ERROR if validation fails or exception occurs
 */
CallbackReturn PLCController::on_configure(const rclcpp_lifecycle::State &)
try
{
  // ========== Load/refresh parameters ==========
  if (!update_dynamic_map_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // ========== Parse interface names from YAML ==========
  store_command_interface_types();
  store_state_interface_types();

  // ========== Validate configuration ==========
  if (command_interface_types_.empty() && state_interface_types_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command or state interfaces are configured");
    return CallbackReturn::ERROR;
  }

  // ========== Create subscription to PlcController commands ==========
  if (!command_interface_types_.empty())
  {
    // Best-effort QoS: drops messages if subscriber falls behind, doesn't block
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    plc_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/plc_commands", qos,
      [this](const CmdType::SharedPtr msg) {
        // Subscription callback: store incoming command in lock-free buffer
        // This callback runs in ROS subscription thread (not real-time)
        rt_command_ptr_.writeFromNonRT(msg);
      });
  }

  // ========== Create publisher for PlcStates feedback ==========
  plc_state_publisher_ =
    get_node()->create_publisher<StateType>("~/plc_states", rclcpp::SystemDefaultsQoS());

  // ========== Create real-time safe publisher wrapper ==========
  realtime_plc_state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StateType>>(plc_state_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
  return CallbackReturn::ERROR;
}


/**
 * Return command interface configuration.
 * 
 * Called by controller_manager::prepare_controller() to determine which
 * GPIO command interfaces this controller needs to be granted access to.
 * 
 * Configuration Type: INDIVIDUAL
 *   Each interface is requested separately (vs. COMBINED would request all interfaces
 *   of a particular joint/GPIO)
 * 
 * @return InterfaceConfiguration with:
 *   - type: interface_configuration_type::INDIVIDUAL
 *   - names: command_interface_types_ (from YAML config)
 */
controller_interface::InterfaceConfiguration
PLCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}


/**
 * Return state interface configuration.
 * 
 * Called by controller_manager::prepare_controller() to determine which
 * GPIO state interfaces this controller needs to be granted read access to.
 * 
 * Configuration Type: INDIVIDUAL
 *   Each interface is requested separately
 * 
 * @return InterfaceConfiguration with:
 *   - type: interface_configuration_type::INDIVIDUAL
 *   - names: state_interface_types_ (from YAML or auto-populated)
 */
controller_interface::InterfaceConfiguration PLCController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  return state_interfaces_config;
}


/**
 * Activate controller for first-time execution (or resume from deactivate).
 * 
 * Lifecycle Stage: Configured → Active
 * 
 * Actions:
 *   1. Create map from command interface names to LoanedCommandInterface references
 *      (called once, references reused in every update() cycle)
 *   2. Create map from state interface names to LoanedStateInterface references
 *   3. Validate all configured interfaces were found in actual interfaces
 *   4. Initialize PlcStates message with interface names and NaN values
 *   5. Reset rt_command_ptr_ buffer (clear any stale command)
 * 
 * Called by: controller_manager when controller transitions to ACTIVE state
 * 
 * Interface Mapping:
 *   - controller_manager provides LoanedCommandInterface and LoanedStateInterface objects
 *   - We create maps to efficiently access them by name in update() loop
 *   - Performed once at activation (not every cycle) for performance
 * 
 * Exception Safety: Catches std::exception during interface creation, logs, returns ERROR
 * 
 * @param previous_state Lifecycle state before transition (unused)
 * @return CallbackReturn::SUCCESS if activation successful
 * @return CallbackReturn::ERROR if interface mismatch detected
 */
CallbackReturn PLCController::on_activate(const rclcpp_lifecycle::State &)
{
  // ========== Create command interface map: name → LoanedCommandInterface ==========
  command_interfaces_map_ =
    create_map_of_references_to_interfaces(command_interface_types_, command_interfaces_);

  // ========== Create state interface map: name → LoanedStateInterface ==========
  state_interfaces_map_ =
    create_map_of_references_to_interfaces(state_interface_types_, state_interfaces_);

  // ========== Validate all configured interfaces were found ==========
  if (
    !check_if_configured_interfaces_matches_received(
      command_interface_types_, command_interfaces_map_) ||
    !check_if_configured_interfaces_matches_received(state_interface_types_, state_interfaces_map_))
  {
    return CallbackReturn::ERROR;
  }

  // ========== Initialize PlcStates message for publishing ==========
  initialize_plc_state_msg();

  // ========== Reset command buffer ==========
  rt_command_ptr_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}


/**
 * Deactivate controller (suspend operation).
 * 
 * Lifecycle Stage: Active → Inactive
 * 
 * Actions:
 *   1. Reset rt_command_ptr_ buffer to release reference to last command
 *   2. Stop processing new commands in next update() cycle
 * 
 * Called by: controller_manager when controller is being suspended or stopped
 * 
 * @param previous_state Lifecycle state before transition (unused)
 * @return CallbackReturn::SUCCESS always
 */
CallbackReturn PLCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  rt_command_ptr_.reset();
  return CallbackReturn::SUCCESS;
}


// ========== Real-time Update Loop ==========

/**
 * Main update loop called at 500 Hz by controller_manager.
 * 
 * This is the real-time critical function called at fixed rate (500 Hz = 2 ms period).
 * It implements the bidirectional GPIO bridge between ROS and PLC.
 * 
 * Execution Sequence:
 *   1. update_plc_states(): Read GPIO input state, publish PlcStates message
 *   2. update_plc_commands(): Read latest PlcController command, apply to GPIO outputs
 * 
 * Real-time Safety:
 *   - No blocking operations (no mutex locks)
 *   - No dynamic memory allocation
 *   - Lock-free RealtimeBuffer for command passing
 *   - Lock-free RealtimePublisher for state publishing
 *   - Non-blocking trylock() on publisher (skips publish if busy, no priority inversion)
 * 
 * Execution Time: < 1 ms typical on dedicated core
 * 
 * @param time Current ROS simulation/system time
 * @param period Duration since last update (nominally 0.002 s at 500 Hz)
 * @return return_type::OK if successful
 */
controller_interface::return_type PLCController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // ========== Read PLC state and publish ==========
  update_plc_states();

  // ========== Apply latest command to PLC ==========
  return update_plc_commands();
}


// ========== Private Methods ==========

/**
 * Refresh dynamic parameters from listener.
 * 
 * Called during on_configure() to load parameters.
 * Parameters can be marked as "dynamic" to allow runtime updates.
 * 
 * @return bool: Currently always returns true (no validation)
 */
bool PLCController::update_dynamic_map_parameters()
{
  auto logger = get_node()->get_logger();
  // Refresh parameters from YAML (handles dynamic parameter updates)
  param_listener_->refresh_dynamic_parameters();
  // Store refreshed parameters
  params_ = param_listener_->get_params();
  return true;
}


/**
 * Parse command interface names from YAML parameters.
 * 
 * Converts params_.command_interfaces (map of GPIO → interface lists) into
 * a flat vector of full interface names: "gpio_name/interface_name".
 * 
 * Example Input (from plc_controller.yaml):
 *   command_interfaces:
 *     PLC_node:
 *       - interfaces: [estop, sonar_teach, force_sensors_pwr, ...]
 * 
 * Example Output:
 *   command_interface_types_ = ["PLC_node/estop", "PLC_node/sonar_teach", ...]
 * 
 * Result stored in: command_interface_types_
 */
void PLCController::store_command_interface_types()
{
  command_interface_types_ = parameter_utils::flatten_gpio_interface_map(params_.command_interfaces.gpios_map);
}


/**
 * Check if state interface auto-broadcast mode should be enabled.
 * 
 * Auto-broadcast mode: If no state interfaces explicitly configured,
 * automatically broadcast all available state interfaces from URDF.
 * 
 * Returns true if ALL configured GPIOs have empty interface lists
 * (indicating auto-broadcast mode desired).
 * 
 * @return bool: true if should auto-broadcast all available, false if explicit list
 */
bool PLCController::should_broadcast_all_interfaces_of_configured_gpios() const
{
  // auto are_interfaces_empty = [](const auto & interfaces)
  // { return interfaces.second.interfaces.empty(); };
  return parameter_utils::all_interface_lists_empty(params_.state_interfaces.gpios_map);
}


/**
 * Extract GPIO definitions from URDF parsed by controller_manager.
 * 
 * Calls hardware_interface API to get all GPIO component info from URDF
 * (includes both command and state interfaces for each GPIO).
 * 
 * @return Vector of ComponentInfo containing all GPIO definitions
 *         Returns empty vector on exception
 */
std::vector<hardware_interface::ComponentInfo> PLCController::get_gpios_from_urdf() const
try
{
  return extract_gpios_from_hardware_info(
    hardware_interface::parse_control_resources_from_urdf(get_robot_description()));
}
catch (const std::exception & e)
{
  fprintf(stderr, "Exception thrown during extracting gpios info from urdf %s \n", e.what());
  return {};
}


/**
 * Auto-populate state interface list with all available GPIO state interfaces.
 * 
 * Called when state_interfaces config is empty (auto-broadcast mode).
 * 
 * For each GPIO in params_.gpios:
 *   1. Find matching GPIO in URDF via get_gpios_from_urdf()
 *   2. Extract all state interface names from URDF GPIO definition
 *   3. Construct full names: "gpio_name/interface_name"
 *   4. Add to state_interface_types_
 * 
 * Result stored in: state_interface_types_
 */
InterfacesNames PLCController::set_all_state_interfaces_of_configured_gpios()
{
  InterfacesNames result;
  const auto gpios{get_gpios_from_urdf()};
  for (const auto & gpio_name : params_.gpios)
  {
    for (const auto & gpio : gpios)
    {
      if (gpio_name == gpio.name)
      {
        std::transform(
          gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
          std::back_insert_iterator(result),
          [&gpio_name](const auto & interface_name)
          { return gpio_name + '/' + interface_name.name; });
      }
    }
  }
  return result;
}


/**
 * Parse state interface names from YAML or auto-populate if not configured.
 * 
 * Two modes:
 * 
 *   1. Explicit Configuration (state_interfaces configured in YAML):
 *      Similar to store_command_interface_types(), parse YAML and construct names
 *   
 *   2. Auto-Broadcast Mode (state_interfaces is empty in YAML):
 *      Call set_all_state_interfaces_of_configured_gpios() to populate from URDF
 * 
 * Result stored in: state_interface_types_
 */
void PLCController::store_state_interface_types()
{
  if (should_broadcast_all_interfaces_of_configured_gpios())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "State interfaces are not configured. All available interfaces of configured GPIOs will be "
      "broadcasted.");
    state_interface_types_ = set_all_state_interfaces_of_configured_gpios();
    return;
  }

  state_interface_types_ = parameter_utils::flatten_gpio_interface_map(params_.state_interfaces.gpios_map);
}


/**
 * Initialize PlcStates message with interface names and NaN placeholder values.
 * 
 * Called during on_activate() before entering update() loop.
 * Pre-populates the message that will be published every cycle.
 * 
 * Initialization:
 *   - interface_names: Extracted from state_interface_types_
 *   - values: Vector of quiet_NaN() (invalid until first real state read)
 * 
 * This pre-allocation ensures no dynamic allocation happens in update() loop.
 */
void PLCController::initialize_plc_state_msg()
{
  auto & plc_state_msg = realtime_plc_state_publisher_->msg_;
  const auto gpio_name = params_.gpios.front();
  plc_state_msg.interface_names = get_plc_state_interfaces_names(gpio_name);
  plc_state_msg.values = std::vector<bool>(
    plc_state_msg.interface_names.size(), false);
}


/**
 * Extract state interface names for a specific GPIO.
 * 
 * Searches state_interfaces_map_ for entries matching the given GPIO name
 * and returns just the interface portion (without the "gpio_name/" prefix).
 * 
 * Example:
 *   gpio_name = "PLC_node"
 *   state_interfaces_map_ contains: "PLC_node/estop", "PLC_node/reset", ...
 *   Returns: ["estop", "reset", ...]
 * 
 * @param gpio_name Name of GPIO to extract interfaces for
 * @return Vector of interface names (interface portion only)
 */
InterfacesNames PLCController::get_plc_state_interfaces_names(
  const std::string & gpio_name) const
{
  InterfacesNames result;
  for (const auto & interface_name : state_interface_types_)
  {
    const auto it = state_interfaces_map_.find(interface_name);
    if (it != state_interfaces_map_.cend() && it->second.get().get_prefix_name() == gpio_name)
    {
      result.emplace_back(it->second.get().get_interface_name());
    }
  }
  return result;
}


// ========== Template Method Implementations ==========

/**
 * Create map from interface names to LoanedInterface references.
 * 
 * Generic template for creating either command or state interface maps.
 * Matches each configured interface name against actual LoanedInterface objects
 * provided by controller_manager, storing references in an unordered_map.
 * 
 * Used by:
 *   - on_activate(): Called twice, once for commands, once for states
 * 
 * @tparam T: LoanedCommandInterface or LoanedStateInterface
 * @param interfaces_from_params Vector of interface names from YAML
 * @param configured_interfaces Vector of actual LoanedInterface objects
 * @return Map: interface_name → reference to matching LoanedInterface
 */
template <typename T>
std::unordered_map<std::string, std::reference_wrapper<T>>
PLCController::create_map_of_references_to_interfaces(
  const InterfacesNames & interfaces_from_params, std::vector<T> & configured_interfaces)
{
  std::unordered_map<std::string, std::reference_wrapper<T>> map;
  for (const auto & interface_name : interfaces_from_params)
  {
    // Find first interface in configured_interfaces matching interface_name
    auto interface = std::find_if(
      configured_interfaces.begin(), configured_interfaces.end(),
      [&](const auto & configured_interface)
      {
        const auto full_name_interface_name = configured_interface.get_name();
        return full_name_interface_name == interface_name;
      });

    // If found, add to map
    if (interface != configured_interfaces.end())
    {
      map.emplace(interface_name, std::ref(*interface));
    }
  }
  return map;
}


/**
 * Validate that all configured interfaces were found in actual interfaces.
 * 
 * Generic template for validating either command or state interfaces.
 * Checks:
 *   1. Count matches: size of configured_interfaces == size of interfaces_from_params
 *   2. All expected names were found (no missing interfaces)
 * 
 * Logs detailed error if mismatch:
 *   - Expected count vs actual count
 *   - List of expected interface names
 *   - List of actual interface names found
 * 
 * @tparam T: LoanedCommandInterface or LoanedStateInterface
 * @param interfaces_from_params Expected interface names from YAML
 * @param configured_interfaces Actual LoanedInterface objects from controller_manager
 * @return bool: true if counts and names match, false otherwise
 */
template <typename T>
bool PLCController::check_if_configured_interfaces_matches_received(
  const InterfacesNames & interfaces_from_params, const T & configured_interfaces)
{
  if (!(configured_interfaces.size() == interfaces_from_params.size()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %ld interfaces, got %ld", interfaces_from_params.size(),
      configured_interfaces.size());

    // Log all expected interfaces
    for (const auto & interface : interfaces_from_params)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %s", interface.c_str());
    }

    // Log all actual interfaces found
    print_interface(get_node()->get_logger(), configured_interfaces);
    return false;
  }
  return true;
}


// ========== Command and State Update Functions ==========

/**
 * Read latest PlcController command and apply to GPIO output registers.
 * 
 * Real-time Context: Called from update() at 500 Hz
 * 
 * Execution Sequence:
 *   1. Read latest command from rt_command_ptr_ (lock-free RealtimeBuffer)
 *      - subscription thread updates this via writeFromNonRT()
 *      - update thread reads via readFromRT()
 *      - Always gets latest (drops intermediate if behind)
 *   
 *   2. If command exists and is valid:
 *      a. Validate interface_names.size() == values.size()
 *         (if not, log error and return)
 *      b. For each command interface in command_interfaces_map_:
 *         - Find this interface name in gpio_commands.interface_names
 *         - Get corresponding value from gpio_commands.values[index]
 *         - Call set_value() on LoanedCommandInterface
 *      c. Log successful publish
 *      d. Return OK
 *   
 *   3. If no command available (rt_command_ptr_ is nullptr):
 *      - Return OK (graceful degradation)
 * 
 * Missing Interfaces:
 *   - If an interface name in command_interfaces_map_ is not found in the command message,
 *     we skip it (commented-out WARN log) and continue
 *   - This allows partial updates (not all interfaces must be present in every command)
 * 
 * Command Message Structure:
 *   PlcController message:
 *     interface_names: ["PLC_node/estop", "PLC_node/sonar_teach", ...]
 *     values: [1, 0, 1, ...]  (corresponding values for each interface)
 * 
 * Exception Handling:
 *   - Catches std::exception and logs via fprintf
 *   - Returns ERROR to stop further processing
 *   - Non-blocking: exception handling doesn't prevent update() from completing
 * 
 * @return return_type::OK if processed successfully
 * @return return_type::ERROR if exception during processing
 */
controller_interface::return_type PLCController::update_plc_commands()
{
  // ========== Read latest command from lock-free buffer ==========
  auto gpio_commands_ptr = rt_command_ptr_.readFromRT();

  // ========== If no command available, return gracefully ==========
  if (!gpio_commands_ptr || !(*gpio_commands_ptr))
  {
    return controller_interface::return_type::OK;
  }

  const auto gpio_commands = *(*gpio_commands_ptr);
  const auto & gpio_name = params_.gpios.front();

  // ========== Validate command message structure ==========
  if (gpio_commands.values.size() != gpio_commands.interface_names.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), " %s interfaces_names do not match the values",
      gpio_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  try
  {
    // ========== Apply command to each GPIO output interface ==========
    for (const auto & [interface_name, command_interface] : command_interfaces_map_)
    {
      // Find this interface name in the command message
      auto it = std::find(
        gpio_commands.interface_names.begin(), gpio_commands.interface_names.end(),
        interface_name);

      // If found, apply the corresponding value
      if (it != gpio_commands.interface_names.end())
      {
        auto index = std::distance(gpio_commands.interface_names.begin(), it);
        [[maybe_unused]] auto result = command_interface.get().set_value(static_cast<double>(gpio_commands.values[index]));
      }
      // If not found, skip (interface not present in this command message)
      // This allows partial updates where not all interfaces are commanded
      // (uncommented: RCLCPP_WARN(get_node()->get_logger(), ...)
    }
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during applying command stage with message: %s \n",
      e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}


/**
 * Read GPIO input state and publish PlcStates message.
 * 
 * Real-time Context: Called from update() at 500 Hz
 * 
 * Execution Sequence:
 *   1. Try to acquire realtime_plc_state_publisher_ lock (non-blocking)
 *      - If locked (previous message still being serialized), return immediately
 *      - This prevents blocking and priority inversion
 *   
 *   2. Clear previous interface_names and values from message
 *   
 *   3. For each state interface in state_interface_types_:
 *      a. Find interface in state_interfaces_map_
 *      b. Read value via get_optional<double>()
 *      c. Append interface_name to plc_state_msg.interface_names
 *      d. Append value to plc_state_msg.values
 *         (or NaN if get_optional has no value)
 *   
 *   4. Release lock and publish asynchronously
 *      - Publish happens in background thread (doesn't block update() loop)
 *      - Latest message always wins if publish thread falls behind
 * 
 * Message Structure:
 *   PlcStates message (published every cycle):
 *     interface_names: ["estop", "reset", "manual_switch_pressed", ...]
 *     values: [1, 0, 1, ...]  (corresponding GPIO input states)
 * 
 * Non-blocking Behavior:
 *   - If publisher lock is held (previous publish still in progress):
 *     → skip this cycle, return immediately
 *     → no blocking, no priority inversion
 *   - If lock acquired:
 *     → update message, release lock
 *     → publish happens asynchronously in publisher's thread
 * 
 * Exception Handling:
 *   - Catches std::exception and logs via fprintf
 *   - Still calls unlockAndPublish() to release lock
 * 
 * @return void (no return value)
 */
void PLCController::update_plc_states()
{
  // ========== Try to acquire publisher lock (non-blocking) ==========
  if (!realtime_plc_state_publisher_ || !realtime_plc_state_publisher_->trylock())
  {
    // Publisher busy (previous message still being serialized), skip this cycle
    return;
  }

  auto & plc_state_msg = realtime_plc_state_publisher_->msg_;
  try
  {
    // ========== Clear previous state ==========
    plc_state_msg.values.clear();
    plc_state_msg.interface_names.clear();

    // ========== Read all GPIO input state registers ==========
    for (const auto & interface_name : state_interface_types_)
    {
      auto it = state_interfaces_map_.find(interface_name);
      if (it != state_interfaces_map_.end())
      {
        const auto & state_interface = it->second.get();

        // ========== Append interface name ==========
        plc_state_msg.interface_names.push_back(state_interface.get_interface_name());

        // ========== Read value from GPIO input register ==========
        auto optional_value = state_interface.get_optional<double>();
        if (optional_value.has_value())
        {
          // Value available: append to message
          plc_state_msg.values.push_back(optional_value.value());
        }
        else
        {
          // Value not available: append NaN placeholder
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to retrieve state value for %s",
            interface_name.c_str());
          plc_state_msg.values.push_back(false);
        }
      }
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during reading state interface, error: %s \n", e.what());
  }

  // ========== Release lock and publish asynchronously ==========
  realtime_plc_state_publisher_->unlockAndPublish();
}


}  // namespace plc_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  plc_controller::PLCController, controller_interface::ControllerInterface)
