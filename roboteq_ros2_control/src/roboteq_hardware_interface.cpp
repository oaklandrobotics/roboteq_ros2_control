// Internal headers
#include "roboteq_ros2_control/socket_can.hpp"
#include "roboteq_ros2_control/can_helpers.hpp"
#include "roboteq_ros2_control/canopen_enums.hpp"
#include "roboteq_ros2_control/canopen_sdo.hpp"
#include "roboteq_ros2_control/roboteq_object_dictionary.hpp"

// ROS2 headers
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"


namespace roboteq_ros2_control
{
  struct Axis
  {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, double gear_ratio) : can_intf_(can_intf), node_id_(node_id), gear_ratio_(gear_ratio) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);
    void send_can_msg(const can_frame& frame) const {
      can_intf_->send_can_frame(frame);
    }

    SocketCanIntf* can_intf_;
    uint8_t node_id_;
    
    double gear_ratio_;

    // Commands (ros2_control => Roboteq)
    double vel_setpoint_ = 0.0f; // [rad/s]
    // double pos_setpoint_ = 0.0f; // [rad] // TODO: Future implementation
    // double torque_setpoint_ = 0.0f; // [Nm] // TODO: Future implementation

    // State (Roboteq => ros2_control)
    double vel_estimate_ = NAN; // [rad/s]
    double pos_estimate_ = NAN; // [rad]
    // double torque_target_ = NAN; // [Nm] // TODO: Future implementation
    // double torque_estimate_ = NAN; // [Nm] // TODO: Future implementation

    bool motor_enabled_ = false;
    bool faulted_ = false;
    bool heartbeat_seen_ = false;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // bool pos_input_enabled_ = false; // TODO: Future implementation
    // bool vel_input_enabled_ = false; // TODO: Future implementation
    // bool torque_input_enabled_ = false; // TODO: Future implementation
  };

  class RoboteqHardwareInterface final : public hardware_interface::SystemInterface
  {
    public:
      using return_type = hardware_interface::return_type;
      using State = rclcpp_lifecycle::State;

      // ROS2 control transitions
      CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
      CallbackReturn on_configure(const State& previous_state) override;
      CallbackReturn on_cleanup(const State& previous_state) override;
      CallbackReturn on_activate(const State& previous_state) override;
      CallbackReturn on_deactivate(const State& previous_state) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
      return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

      void reinitialize();
      void estop();

    private:
      void on_can_msg(const can_frame& frame);

      EpollEventLoop event_loop_;
      std::vector<Axis> axes_;
      std::string can_intf_name_;
      SocketCanIntf can_intf_;
      rclcpp::Time timestamp_;

      std::atomic<bool> estop_active_ = false;

      // The ratio for our current setup is 7.3:1
      double gear_ratio_ = 7.3;

      // For reinitializing the control
      std::shared_ptr<rclcpp::Node> service_node_;
      std::thread spin_thread_;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinit_srv_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  };
} // namespace roboteq_ros2_control

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
    axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")), gear_ratio_);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_configure(const State&)
{
  // For various service controls
  service_node_ = rclcpp::Node::make_shared("roboteq_service_listener");

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

  // Spin up the node
  spin_thread_ = std::thread([this]() { rclcpp::spin(service_node_); });
  // spin_thread_.detach();

  RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Spinning service node in background thread.");

  if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&RoboteqHardwareInterface::on_can_msg, this, _1)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RoboteqHardwareInterface"),
      "Failed to initialize SocketCAN on %s",
      can_intf_name_.c_str()
    );
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RoboteqHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboteqHardwareInterface::on_cleanup(const State&)
{
  // Cleanup the reinitialization node
  if (service_node_)
  {
      service_node_->get_node_base_interface()->get_context()->shutdown("Shutting down Roboteq service listener.");
  }

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
    auto vel_can_frame = canopen::build_pdo_message(axis.node_id_, canopen::COBID::RPDO1, inputVel);
    axis.send_can_msg(vel_can_frame);

    // TODO: Implement additional interfaces
  }

  return return_type::OK;
}

void RoboteqHardwareInterface::on_can_msg(const can_frame& frame)
{
  for (auto& axis : axes_)
  {
    if ((frame.can_id >> 5) == axis.node_id_)
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
      double posEstimate = static_cast<double>(pos_raw);
      double velEstimate = static_cast<double>(vel_raw);

      // Our encoder is before the gearbox, we will be handling the gearbox ratio here.
      posEstimate /= gear_ratio_;
      velEstimate /= gear_ratio_;
      
      pos_estimate_ = posEstimate * (2 * M_PI);
      vel_estimate_ = velEstimate * (2 * M_PI);
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