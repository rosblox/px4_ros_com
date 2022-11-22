#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <control_toolbox/pid_ros.hpp>

#include <math.h>
#include <chrono>
#include <iostream>

const double PI = std::atan(1.0)*4;

const int X_BUTTON = 1;
const int TRIANGLE_BUTTON = 3;
const int R1_BUTTON = 5;
const int R2_BUTTON = 7;
const int L1_BUTTON = 4;
const int L2_BUTTON = 6;

const int LEFT_BUTTON = 6;
const int RIGHT_BUTTON = 6;

const double MIN_THRUST = 0.0;
const double MAX_THRUST = 0.6;
// MAX 0.3 -> 5A

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


		enable_thrust_ = true;

		setpoint_.x = 0.0;
		setpoint_.y = 0.05;
		setpoint_.z = 0.0;

		// publish_setpoint_and_arm();

		vehicle_odometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/VehicleOdometry", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					this->vehicle_orientation_ = msg->q;
				});

		setpoint_sub_ =
			this->create_subscription<geometry_msgs::msg::Vector3>("setpoint", 10,
				[this](const geometry_msgs::msg::Vector3::UniquePtr msg) {
					setpoint_ = *msg;
				});
		
		this->declare_parameter<double>("pid.p", 0.001);
		this->declare_parameter<double>("pid.i", 0.0);
		this->declare_parameter<double>("pid.d", 0.0005);
		this->declare_parameter<double>("pid.i_clamp_max", 0.1);
		this->declare_parameter<double>("pid.i_clamp_min", -0.1);
		this->declare_parameter<bool>("pid.antiwindup", true);

		ptr = std::shared_ptr<rclcpp::Node>(this);
		pid = std::make_shared<control_toolbox::PidROS>(ptr, "pid");
		pid->initPid();


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if(actuator_motors_publisher_->get_subscription_count() == 0 ||
			   vehicle_command_publisher_->get_subscription_count() == 0 ||
			   offboard_control_mode_publisher_->get_subscription_count() == 0){	
				auto clk = *this->get_clock();
				RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 500, "Motors subscriber %ld:", actuator_motors_publisher_->get_subscription_count());
				RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 500, "VehicleCommand subscriber %ld:", vehicle_command_publisher_->get_subscription_count());
				RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 500, "ControlMode subscriber %ld:", offboard_control_mode_publisher_->get_subscription_count());
				return;
			}


			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);


				// Arm the vehicle
				this->arm();
			}


			publish_offboard_control_mode();
			publish_actuator_motors();
		
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};

		period_ = 10ms;
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
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr setpoint_sub_;

	std::shared_ptr<rclcpp::Node> ptr;
	std::shared_ptr<control_toolbox::PidROS> pid;

	std::array<float, 4> vehicle_orientation_;
	geometry_msgs::msg::Vector3 setpoint_;

	bool enable_thrust_;
	int offboard_setpoint_counter_;
	
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


// RCLCPP_INFO(this->get_logger(), "yaw_calculated: %f", yaw_measured_deg);
// RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "thrust_left saturated");

void OffboardControl::publish_actuator_motors() const {

	auto clk = *this->get_clock();

	if(!enable_thrust_){
		ActuatorMotors msg{};
		msg.control = {0.0, 0.0};
		actuator_motors_publisher_->publish(msg);
		return;
	}



	// Yaw error
	double yaw_measured_deg = quaternion_get_yaw(vehicle_orientation_) * 180.0/PI;

	auto error = setpoint_.x - yaw_measured_deg;

	if(error > 180.0){
		error-=360.0;
	} else if (error < -180.0){
		error+=360.0;
	}

	double yaw_thrust = pid->computeCommand(error, period_);

	double yaw_thrust_left = yaw_thrust < 0.0 ? 0.0 : yaw_thrust;
	double yaw_thrust_right = yaw_thrust < 0.0 ? std::fabs(yaw_thrust) : 0.0;


	// Thrust
	double thrust_left = setpoint_.y + yaw_thrust_left;
	double thrust_right = setpoint_.y + yaw_thrust_right;


	// Clamping
	if(thrust_left > MAX_THRUST){
		thrust_right -= (thrust_left-MAX_THRUST);
		thrust_left = MAX_THRUST;
	}
	else if (thrust_right > MAX_THRUST){
		thrust_left -= (thrust_right-MAX_THRUST);
		thrust_right = MAX_THRUST;
	}

	thrust_left = std::clamp(thrust_left, MIN_THRUST, MAX_THRUST);
	thrust_right = std::clamp(thrust_right, MIN_THRUST, MAX_THRUST);


	RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 10000, "(error, left,right): (%f, %f, %f)", error, thrust_left, thrust_right);

	// Combine the two
	ActuatorMotors msg{};

	msg.control = {(float) thrust_left, (float) thrust_right};

	actuator_motors_publisher_->publish(msg);
}


void OffboardControl::publish_setpoint_and_arm() const {

	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    std::this_thread::sleep_for(50ms);

	arm();
    std::this_thread::sleep_for(50ms);


	for (int i = 0; i < 10; i++) {
		publish_offboard_control_mode();
    	// std::this_thread::sleep_for(30ms);
		// ActuatorMotors msg{};
		// msg.control = {(float) setpoint_.y, (float) setpoint_.y};
		actuator_motors_publisher_->publish(ActuatorMotors());
    	// std::this_thread::sleep_for(30ms);
	}
	


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
