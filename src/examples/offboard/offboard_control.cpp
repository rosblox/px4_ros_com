#include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
// #include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {

		using rclcpp::contexts::get_global_default_context;
		get_global_default_context()->add_pre_shutdown_callback(
		[this]() {
			this->timer_->cancel();
			std::this_thread::sleep_for(50ms);
			this->disarm();
		});

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/in/OffboardControlMode", 10);
		actuator_motors_publisher_ =
			this->create_publisher<ActuatorMotors>("fmu/in/ActuatorMotors", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/in/VehicleCommand", 10);

		vehicle_odometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/VehicleOdometry", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					this->vehicle_odometry_ = msg->q;
					RCLCPP_INFO(this->get_logger(), "%f, %f", msg->q[0], msg->q[3]);
				});

		publish_setpoint_and_arm();

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			// publish_offboard_control_mode();
			publish_actuator_motors();
			offboard_setpoint_counter_ ++;
		};


		timer_ = this->create_wall_timer(5ms, timer_callback);
	}

	void arm() const;
	void disarm() const;
	void publish_setpoint_and_arm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

	// std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	std::array<float, 4> vehicle_odometry_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_actuator_motors() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
};




void OffboardControl::publish_actuator_motors() const {
	ActuatorMotors msg{};

	float thrust_x = abs(offboard_setpoint_counter_ % 100 - 50.0)/500.0;
	thrust_x = thrust_x <= 0.035 ? 0.035 : thrust_x;

	// msg.timestamp = timestamp_.load();
	thrust_x = 0.1;
	msg.control = {thrust_x, thrust_x, 0,0,0,0,0,0,0,0,0,0};

	actuator_motors_publisher_->publish(msg);
}


void OffboardControl::publish_setpoint_and_arm() const {

	for (int i = 0; i < 10; i++) {
		publish_offboard_control_mode();
    	std::this_thread::sleep_for(30ms);
		actuator_motors_publisher_->publish(ActuatorMotors());
    	std::this_thread::sleep_for(30ms);
	}
	
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    std::this_thread::sleep_for(50ms);

	arm();
    std::this_thread::sleep_for(50ms);
}


void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	// msg.timestamp = timestamp_.load();
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = true;
	
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) const {
	VehicleCommand msg{};
	// msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	std::shared_ptr offboard_control = std::make_shared<OffboardControl>();

	rclcpp::spin(offboard_control);

	rclcpp::shutdown();
	return 0;
}
