#include "../include/roboteq_ros2_control/roboteq_hardware_interface.hpp"

using namespace roboteq_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn RoboteqHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  estop_active_ = false;

  can_intf_name_ = info_.hardware_parameters["can"];

  for (auto& joint : info_.joints)
  {
    axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")), std::stod(joint.parameters.at("gear_ratio")));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_configure(const State&)
{
  if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&RoboteqHardwareInterface::on_can_msg, this, _1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("RoboteqHardwareInterface"),
      "Failed to initialize SocketCAN on %s",
      can_intf_name_.c_str()
    );
    return CallbackReturn::FAILURE;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());

  // For various service controls
  service_node_ = rclcpp::Node::make_shared("roboteq_service_listener");
  service_executor_.add_node(service_node_);

  reinit_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/roboteq/reinit",
    [this] (const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        this->reinitialize();
        res->success = true;
        res->message = "Reinitialization Complete";
    }
  );

  estop_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/roboteq/estop",
    [this] (const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        this->estop();
        res->success = true;
        res->message = "Estop Activated";
    }
  );

  // Spin up executor
  spin_thread_ = std::thread(
    [this]()
    {
      service_executor_.spin();
    }
  );

  RCLCPP_INFO(
    rclcpp::get_logger("RoboteqHardwareInterface"),
    "Service executor started."
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_cleanup(const State&)
{
  // Cleanup the reinitialization node
  service_executor_.cancel();

  if (spin_thread_.joinable())
  {
      spin_thread_.join();
  }

  can_intf_.deinit();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_activate(const State&) {
  RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Activating Roboteq controllers...");

  // Set NMT to active
  can_frame NMTActiveFrame {};
  NMTActiveFrame.can_id = static_cast<canid_t>(canopen::COBID::NMT);
  NMTActiveFrame.can_dlc = 1;
  NMTActiveFrame.data[0] = static_cast<uint8_t>(canopen::NMT::goToOperational);
  axes_[0].send_can_msg(NMTActiveFrame);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_deactivate(const State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Deactivating Roboteq controllers...");

  // Set NMT to Pre Operational
  can_frame NMTActiveFrame {};
  NMTActiveFrame.can_id = static_cast<canid_t>(canopen::COBID::NMT);
  NMTActiveFrame.can_dlc = 1;
  NMTActiveFrame.data[0] = static_cast<uint8_t>(canopen::NMT::goToPreOperational);
  axes_[0].send_can_msg(NMTActiveFrame);

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_error(const State&)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RoboteqHardwareInterface"),
    "Error occurred during operation"
  );

  // Try to set NMT to pre-operational
  try
  {
    can_frame NMTActiveFrame {};
    NMTActiveFrame.can_id = static_cast<canid_t>(canopen::COBID::NMT);
    NMTActiveFrame.can_dlc = 1;
    NMTActiveFrame.data[0] = static_cast<uint8_t>(canopen::NMT::goToPreOperational);
    axes_[0].send_can_msg(NMTActiveFrame);
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RoboteqHardwareInterface"),
      "CAN interface unable to set NMT to Pre-Operational."
    );
  }

  // Clean up service
  service_executor_.cancel();

  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }

  if (service_node_)
  {
    service_executor_.remove_node(service_node_);
  }

  // Release service pointers
  reinit_srv_.reset();
  estop_srv_.reset();
  service_node_.reset();

  // Release CAN interface
  can_intf_.deinit();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboteqHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &axes_[i].vel_estimate_
    ));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &axes_[i].pos_estimate_
    ));
    // TODO: Implement additional interfaces
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboteqHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &axes_[i].vel_setpoint_
    ));
    // TODO: Implement additional interfaces
  }

  return command_interfaces;
}

return_type RoboteqHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&)
{
  timestamp_ = timestamp;

  while (can_intf_.read_nonblocking()) {
      // repeat until CAN interface has no more messages
  }

  return return_type::OK;
}

return_type RoboteqHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (estop_active_)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("RoboteqHardwareInterface"), "Estop is active. Skipping write.");
    return return_type::OK;
  }

  for (auto& axis : axes_)
  {
    std::int32_t inputVel = (axis.vel_setpoint_ * gear_ratio_) / (2 * M_PI);
    inputVel *= 60;
    auto vel_can_frame = canopen::build_pdo_message(axis.node_id_, canopen::COBID::RPDO1, inputVel);
    axis.send_can_msg(vel_can_frame);

    // TODO: Implement additional interfaces
  }

  return return_type::OK;
}

void RoboteqHardwareInterface::on_can_msg(const can_frame& frame)
{
  const std::uint16_t cob_id = static_cast<std::uint16_t>(frame.can_id & CAN_SFF_MASK);
  const std::uint8_t node_id = static_cast<std::uint8_t>(cob_id & 0x7F);

  for (auto& axis : axes_)
  {
    if (node_id == axis.node_id_)
    {
      axis.on_can_msg(timestamp_, frame);
    }
  }
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame)
{
  uint16_t cob_id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
  uint16_t function_code = cob_id & 0x780;

  // Reconstruct data frame LSB is in first data frame
  int32_t pos_raw = (static_cast<std::uint32_t>(frame.data[7]) << 24) |
                      (static_cast<std::uint32_t>(frame.data[6]) << 16) |
                      (static_cast<std::uint32_t>(frame.data[5]) << 8)  |
                      (static_cast<std::uint32_t>(frame.data[4]));

  int32_t vel_raw = (static_cast<std::uint32_t>(frame.data[3]) << 24) |
                    (static_cast<std::uint32_t>(frame.data[2]) << 16) |
                    (static_cast<std::uint32_t>(frame.data[1]) << 8)  |
                    (static_cast<std::uint32_t>(frame.data[0]));

  switch (function_code)
  {
    case static_cast<uint16_t>(canopen::COBID::TPDO1):
    {
      int32_t invert = 1;

      double posEstimate = static_cast<double>(pos_raw);
      double velEstimate = static_cast<double>(vel_raw);

      // Our encoder is before the gearbox, we will be handling the gearbox ratio here.
      posEstimate /= 7800;
      velEstimate /= gear_ratio_;

      
      pos_estimate_ = (invert) * posEstimate * (2 * M_PI);
      vel_estimate_ = (invert) * velEstimate * (2 * M_PI) / 60;
    } break;
    // TODO: Implement Torque Feedback
    // silently ignore unimplemented command IDs
  }
}

// For reinitializing control
void RoboteqHardwareInterface::reinitialize()
{
    if (estop_active_)
    {
        estop_active_ = false;
        RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Clearing estop...");
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Reinitializing Roboteq controllers...");

    for (auto& axis : axes_)
    {
      auto emerShutdown = canopen::build_sdo_write_request(axis.node_id_, roboteq::EmergencyShutdown, 0x00); // WARNING: PLACEHOLDER VALUE
      axis.send_can_msg(emerShutdown);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    for (auto& axis : axes_)
    {
      auto releaseShutdown = canopen::build_sdo_write_request(axis.node_id_, roboteq::ReleaseShutdown, 0x00); // WARNING: PLACEHOLDER VALUE
      axis.send_can_msg(releaseShutdown);
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Roboteq reinit complete.");
}

// For estop
void RoboteqHardwareInterface::estop() {
    estop_active_ = true;

    RCLCPP_WARN(rclcpp::get_logger("RoboteqHardwareInterface"), "Estop activated. Sending Emergency Stop Request to all axes.");

    for (auto& axis : axes_) {
      auto emerShutdown = canopen::build_sdo_write_request(axis.node_id_, roboteq::EmergencyShutdown, 0x00); // WARNING: PLACEHOLDER VALUE
      axis.send_can_msg(emerShutdown);
    }
}

PLUGINLIB_EXPORT_CLASS(roboteq_ros2_control::RoboteqHardwareInterface, hardware_interface::SystemInterface)