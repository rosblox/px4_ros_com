#include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
// #include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

// #include <px4_ros_com/frame_transforms.h>
#include <control_toolbox/pid_ros.hpp>

// control_toolbox::Pid
#include <math.h>
#include <chrono>
#include <iostream>

const double PI = std::atan(1.0)*4;

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
					this->vehicle_orientation_ = msg->q;
					// RCLCPP_INFO(this->get_logger(), "%f, %f", msg->q[0], msg->q[3]);
				});

		setpoint_sub_ =
			this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
				[this](const sensor_msgs::msg::Joy::UniquePtr msg) {
					this->setpoint_ = *msg;
				});

		std::vector<float> initial_axis_setpoints(8, 0.0);
		setpoint_.axes = initial_axis_setpoints;



		ptr = std::shared_ptr<rclcpp::Node>(this);

		this->declare_parameter<double>("pid.p", 0.25);
		this->declare_parameter<double>("pid.i", 0.0);
		this->declare_parameter<double>("pid.d", 0.1);
		this->declare_parameter<double>("pid.i_clamp_max", 0.0);
		this->declare_parameter<double>("pid.i_clamp_min", 0.0);
		this->declare_parameter<bool>("pid.antiwindup", false);

		pid = std::make_shared<control_toolbox::PidROS>(ptr, "pid");
		pid->initPid();

		publish_setpoint_and_arm();


		auto timer_callback = [this]() -> void {
			// publish_offboard_control_mode();
			publish_actuator_motors();
		};

		period_ = 20ms;
		timer_ = this->create_wall_timer(period_, timer_callback);
	}

	void arm() const;
	void disarm() const;
	void publish_setpoint_and_arm() const;

private:
	std::chrono::milliseconds period_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr setpoint_sub_;

	std::shared_ptr<rclcpp::Node> ptr;
	std::shared_ptr<control_toolbox::PidROS> pid;

	std::array<float, 4> vehicle_orientation_;
	sensor_msgs::msg::Joy setpoint_;

	void publish_offboard_control_mode() const;
	void publish_actuator_motors() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
};



static constexpr float sq(float var) { return var * var; }

//Doesn't work: px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(q, roll_measured, pitch_measured, yaw_measured);
double quaternion_get_yaw(std::array<float,4> q){
	const float a = 2.f * (q[0] * q[3] + q[1] * q[2]);
	const float b = sq(q[0]) + sq(q[1]) - sq(q[2]) - sq(q[3]);
	return atan2f(a, b);
}

void OffboardControl::publish_actuator_motors() const {

	// Yaw error
	double yaw_measured_deg;
	yaw_measured_deg = quaternion_get_yaw(vehicle_orientation_) * 180.0/PI;
	// RCLCPP_INFO(this->get_logger(), "yaw_calculated: %f", yaw_measured_deg);

	double yaw_measured_normalized = yaw_measured_deg / 180.0;

	double yaw_setpoint = setpoint_.axes[2];
	double yaw_setpoint_normalized = yaw_setpoint;

	auto error = yaw_setpoint_normalized - yaw_measured_normalized;
	// RCLCPP_INFO(this->get_logger(), "error (normalized, deg): %f %f", error, error * 180.0);

	if(error > 1.0){
		error-=2.0;
	} else if (error < -1.0){
		error+=2.0;
	}
	// RCLCPP_INFO(this->get_logger(), "error after warparound(norm, deg): %f, %f", error, error * 180.0);


	double yaw_thrust = pid->computeCommand(error, period_);
	double yaw_thrust_left = yaw_thrust/2.0;
	double yaw_thrust_right = -yaw_thrust/2.0;



	// This whole section should not be needed...
	const double yaw_thrust_limit = 0.1;

	auto clk = *this->get_clock();
	if(std::fabs(yaw_thrust_left)>yaw_thrust_limit){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "yaw_thrust_left saturated");
	} 
	if(std::fabs(yaw_thrust_right)>yaw_thrust_limit){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "yaw_thrust_right saturated");
	}

	yaw_thrust_left = std::clamp(yaw_thrust_left, -yaw_thrust_limit, yaw_thrust_limit);
	yaw_thrust_right = std::clamp(yaw_thrust_right, -yaw_thrust_limit, yaw_thrust_limit);
	// ... until here.

	// RCLCPP_INFO(this->get_logger(), "yaw_thrust (left, right): %f %f", yaw_thrust_left, yaw_thrust_right);



	double theta_thrust = 0.1;

	double thrust_left = theta_thrust + yaw_thrust_left;
	double thrust_right = theta_thrust + yaw_thrust_right;

	// //maybe this is not needed
	// if(thrust_left < 0.0){
	// 	thrust_right += std::fabs(thrust_left);
	// }
	// if(thrust_right < 0.0){
	// 	thrust_left += std::fabs(thrust_right);
	// }

	// RCLCPP_INFO(this->get_logger(), "thrust (left, right): %f %f", thrust_left, thrust_right);

	const double thrust_limit_low = 0.0;
	const double thrust_limit_high = 0.25; // MAX 0.3 -> 5A

	if(thrust_left>thrust_limit_high){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "thrust_left saturated");
	} 
	if(thrust_left<thrust_limit_low){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "thrust_left below limit");
	} 
	if(thrust_right>thrust_limit_high){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "thrust_right saturated");
	}
	if(thrust_right<thrust_limit_low){
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "thrust_right below limit");
	} 

	thrust_left = std::clamp(thrust_left, thrust_limit_low, thrust_limit_high);
	thrust_right = std::clamp(thrust_right, thrust_limit_low, thrust_limit_high);

	// RCLCPP_INFO(this->get_logger(), "thrust_limited (left, right): %f %f", thrust_left, thrust_right);


	ActuatorMotors msg{};

	msg.control = {(float) thrust_left, (float) thrust_right};

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
