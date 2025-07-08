/* Author: Austin Parks
 *   Date:     07-02-2025
 * Debug republisher node: This node will subscribe to all natively advertised 
 * internal PX4 Firmware topics and republish them under a different namespace
 * using default QoS settings for rqt Topic Monitor plugin compatibility.
 */
#include <rclcpp/rclcpp.hpp>
////////////////////////////////////////////////////////////// PX4 OUT
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/arming_check_request.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/collision_constraints.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/message_format_response.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vtol_vehicle_status.hpp>
////////////////////////////////////////////////////////////// PX4 IN
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/arming_check_reply.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/config_overrides.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/fixed_wing_lateral_setpoint.hpp>
#include <px4_msgs/msg/fixed_wing_longitudinal_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include <px4_msgs/msg/lateral_control_configuration.hpp>
#include <px4_msgs/msg/longitudinal_control_configuration.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/message_format_request.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/onboard_computer_status.hpp>
#include <px4_msgs/msg/register_ext_component_request.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <px4_msgs/msg/telemetry_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/unregister_ext_component.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
//////////////////////////////////////////////////////////////
#include <stdint.h>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <geographic_msgs/msg/geo_point.h>

#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono;

class Px4RosNode : public rclcpp::Node
{
public:
    Px4RosNode();
    
    void arm();
    void disarm();
    
private:
    ///////////////////////////////////////////// PX4 Sub  ////////////////////////////////////////////////////
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated            >::SharedPtr sub_airspeed_validated;
    rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest           >::SharedPtr sub_arming_check_request;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus                >::SharedPtr sub_battery_status;
    rclcpp::Subscription<px4_msgs::msg::CollisionConstraints         >::SharedPtr sub_collision_constraints;
    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags         >::SharedPtr sub_estimator_status_flags;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags                >::SharedPtr sub_failsafe_flags;
    rclcpp::Subscription<px4_msgs::msg::HomePosition                 >::SharedPtr sub_home_position;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint        >::SharedPtr sub_manual_control_setpoint;
    rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse        >::SharedPtr sub_message_format_response;
    rclcpp::Subscription<px4_msgs::msg::ModeCompleted                >::SharedPtr sub_mode_completed;
    rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet      >::SharedPtr sub_position_setpoint_triplet;
    rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply    >::SharedPtr sub_register_ext_component_reply;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined               >::SharedPtr sub_sensor_combined;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus               >::SharedPtr sub_timesync_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude              >::SharedPtr sub_vehicle_attitude;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck            >::SharedPtr sub_vehicle_command_ack;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode           >::SharedPtr sub_vehicle_control_mode;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition        >::SharedPtr sub_vehicle_global_position;
    rclcpp::Subscription<px4_msgs::msg::SensorGps                    >::SharedPtr sub_vehicle_gps_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected          >::SharedPtr sub_vehicle_land_detected;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition         >::SharedPtr sub_vehicle_local_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry              >::SharedPtr sub_vehicle_odometry;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus                >::SharedPtr sub_vehicle_status_v1;
    rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus            >::SharedPtr sub_vtol_vehicle_status;
    ////////////////////////////////////////// PX4 Re-Publish //////////////////////////////////////////////////
    rclcpp::Publisher<px4_msgs::msg::AirspeedValidated               >::SharedPtr rpb_airspeed_validated;
    rclcpp::Publisher<px4_msgs::msg::ArmingCheckRequest              >::SharedPtr rpb_arming_check_request;
    rclcpp::Publisher<px4_msgs::msg::BatteryStatus                   >::SharedPtr rpb_battery_status;
    rclcpp::Publisher<px4_msgs::msg::CollisionConstraints            >::SharedPtr rpb_collision_constraints;
    rclcpp::Publisher<px4_msgs::msg::EstimatorStatusFlags            >::SharedPtr rpb_estimator_status_flags;
    rclcpp::Publisher<px4_msgs::msg::FailsafeFlags                   >::SharedPtr rpb_failsafe_flags;
    rclcpp::Publisher<px4_msgs::msg::HomePosition                    >::SharedPtr rpb_home_position;
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint           >::SharedPtr rpb_manual_control_setpoint;
    rclcpp::Publisher<px4_msgs::msg::MessageFormatResponse           >::SharedPtr rpb_message_format_response;
    rclcpp::Publisher<px4_msgs::msg::ModeCompleted                   >::SharedPtr rpb_mode_completed;
    rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet         >::SharedPtr rpb_position_setpoint_triplet;
    rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentReply       >::SharedPtr rpb_register_ext_component_reply;
    rclcpp::Publisher<px4_msgs::msg::SensorCombined                  >::SharedPtr rpb_sensor_combined;
    rclcpp::Publisher<px4_msgs::msg::TimesyncStatus                  >::SharedPtr rpb_timesync_status;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude                 >::SharedPtr rpb_vehicle_attitude;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck               >::SharedPtr rpb_vehicle_command_ack;
    rclcpp::Publisher<px4_msgs::msg::VehicleControlMode              >::SharedPtr rpb_vehicle_control_mode;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition           >::SharedPtr rpb_vehicle_global_position;
    rclcpp::Publisher<px4_msgs::msg::SensorGps                       >::SharedPtr rpb_vehicle_gps_position;
    rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected             >::SharedPtr rpb_vehicle_land_detected;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition            >::SharedPtr rpb_vehicle_local_position;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry                 >::SharedPtr rpb_vehicle_odometry;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus                   >::SharedPtr rpb_vehicle_status_v1;
    rclcpp::Publisher<px4_msgs::msg::VtolVehicleStatus               >::SharedPtr rpb_vtol_vehicle_status;
    //////////////////////////////////////////// Publish PX4 Offboard Control ///////////////////////////////////
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors                  >::SharedPtr pub_actuator_motors;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos                  >::SharedPtr pub_actuator_servos;
    rclcpp::Publisher<px4_msgs::msg::ArmingCheckReply                >::SharedPtr pub_arming_check_reply_v1;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition           >::SharedPtr pub_aux_global_position;
    rclcpp::Publisher<px4_msgs::msg::VehicleControlMode              >::SharedPtr pub_config_control_setpoints;
    rclcpp::Publisher<px4_msgs::msg::ConfigOverrides                 >::SharedPtr pub_config_overrides_request;
    rclcpp::Publisher<px4_msgs::msg::DistanceSensor                  >::SharedPtr pub_distance_sensor;
    rclcpp::Publisher<px4_msgs::msg::FixedWingLateralSetpoint        >::SharedPtr pub_fixed_wing_lateral_setpoint;
    rclcpp::Publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint   >::SharedPtr pub_fixed_wing_longitudinal_setpoint;
    rclcpp::Publisher<px4_msgs::msg::GotoSetpoint                    >::SharedPtr pub_goto_setpoint;
    rclcpp::Publisher<px4_msgs::msg::LateralControlConfiguration     >::SharedPtr pub_lateral_control_configuration;
    rclcpp::Publisher<px4_msgs::msg::LongitudinalControlConfiguration>::SharedPtr pub_longitudinal_control_configuration;
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint           >::SharedPtr pub_manual_control_input;
    rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest            >::SharedPtr pub_message_format_request;
    rclcpp::Publisher<px4_msgs::msg::ModeCompleted                   >::SharedPtr pub_mode_completed;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance                >::SharedPtr pub_obstacle_distance;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode             >::SharedPtr pub_offboard_control_mode;
    rclcpp::Publisher<px4_msgs::msg::OnboardComputerStatus           >::SharedPtr pub_onboard_computer_status;
    rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentRequest     >::SharedPtr pub_register_ext_component_request;
    rclcpp::Publisher<px4_msgs::msg::SensorOpticalFlow               >::SharedPtr pub_sensor_optical_flow;
    rclcpp::Publisher<px4_msgs::msg::TelemetryStatus                 >::SharedPtr pub_telemetry_status;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint              >::SharedPtr pub_trajectory_setpoint;
    rclcpp::Publisher<px4_msgs::msg::UnregisterExtComponent          >::SharedPtr pub_unregister_ext_component;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint         >::SharedPtr pub_vehicle_attitude_setpoint_v1;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand                  >::SharedPtr pub_vehicle_command;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand                  >::SharedPtr pub_vehicle_command_mode_executor;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry                 >::SharedPtr pub_vehicle_mocap_odometry;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint            >::SharedPtr pub_vehicle_rates_setpoint;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint           >::SharedPtr pub_vehicle_thrust_setpoint;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint           >::SharedPtr pub_vehicle_torque_setpoint;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry                 >::SharedPtr pub_vehicle_visual_odometry;
    
    int spin_cnt;
    px4_msgs::msg::VehicleOdometry       px4_odom;
    px4_msgs::msg::HomePosition          px4_home_pos;
    px4_msgs::msg::VehicleLocalPosition  px4_cpos;
    px4_msgs::msg::VehicleAttitude       px4_catt;
    px4_msgs::msg::VehicleGlobalPosition px4_geo_pos;
    px4_msgs::msg::VehicleStatus         px4_status;
    
    bool px4_odom_good;
    bool px4_home_pos_good;
    bool px4_cpos_good;
    bool px4_catt_good;
    bool px4_geo_pos_good;
    bool px4_status_good;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped                >::SharedPtr pub_ros_pose_enu;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped                >::SharedPtr pub_ros_dbg_pose;
    rclcpp::Publisher<nav_msgs::msg::Odometry                        >::SharedPtr pub_ros_odom;
    
    geometry_msgs::msg::PoseStamped ros_pose_enu;
    geometry_msgs::msg::PoseStamped ros_dbg_pose;
    geographic_msgs::msg::GeoPoint  ros_home_pos;
    nav_msgs::msg::Odometry         ros_odom;
    
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    bool ctrl_en;
    bool ctrl_pos;
    bool ctrl_vel;
    //rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    geometry_msgs::msg::TransformStamped tf_local_utm;
    geometry_msgs::msg::TransformStamped tf_fmu;
    //geometry_msgs::msg::TransformStamped tf_base_link;
    bool tf_good;
    double theta_traj;
    double theta_traj_dx;
    
    void cb_airspeed_validated          (const px4_msgs::msg::AirspeedValidated::SharedPtr         msg){ rpb_airspeed_validated          ->publish(*msg); }
    void cb_arming_check_request        (const px4_msgs::msg::ArmingCheckRequest::SharedPtr        msg){ rpb_arming_check_request        ->publish(*msg); }
    void cb_battery_status              (const px4_msgs::msg::BatteryStatus::SharedPtr             msg){ rpb_battery_status              ->publish(*msg); }
    void cb_collision_constraints       (const px4_msgs::msg::CollisionConstraints::SharedPtr      msg){ rpb_collision_constraints       ->publish(*msg); }
    void cb_estimator_status_flags      (const px4_msgs::msg::EstimatorStatusFlags::SharedPtr      msg){ rpb_estimator_status_flags      ->publish(*msg); }
    void cb_failsafe_flags              (const px4_msgs::msg::FailsafeFlags::SharedPtr             msg){ rpb_failsafe_flags              ->publish(*msg); }
    void cb_home_position               (const px4_msgs::msg::HomePosition::SharedPtr              msg){ rpb_home_position               ->publish(*msg); if(!px4_home_pos_good && px4_cpos_good && px4_catt_good) calc_utm_tf(msg); }
    void cb_manual_control_setpoint     (const px4_msgs::msg::ManualControlSetpoint::SharedPtr     msg){ rpb_manual_control_setpoint     ->publish(*msg); }
    void cb_message_format_response     (const px4_msgs::msg::MessageFormatResponse::SharedPtr     msg){ rpb_message_format_response     ->publish(*msg); }
    void cb_mode_completed              (const px4_msgs::msg::ModeCompleted::SharedPtr             msg){ rpb_mode_completed              ->publish(*msg); }
    void cb_position_setpoint_triplet   (const px4_msgs::msg::PositionSetpointTriplet::SharedPtr   msg){ rpb_position_setpoint_triplet   ->publish(*msg); }
    void cb_register_ext_component_reply(const px4_msgs::msg::RegisterExtComponentReply::SharedPtr msg){ rpb_register_ext_component_reply->publish(*msg); }
    void cb_sensor_combined             (const px4_msgs::msg::SensorCombined::SharedPtr            msg){ rpb_sensor_combined             ->publish(*msg); }
    void cb_timesync_status             (const px4_msgs::msg::TimesyncStatus::SharedPtr            msg){ rpb_timesync_status             ->publish(*msg); }
    void cb_vehicle_attitude            (const px4_msgs::msg::VehicleAttitude::SharedPtr           msg){ rpb_vehicle_attitude            ->publish(*msg); px4_catt = *msg; px4_catt_good = true;}
    void cb_vehicle_command_ack         (const px4_msgs::msg::VehicleCommandAck::SharedPtr         msg){ rpb_vehicle_command_ack         ->publish(*msg); }
    void cb_vehicle_control_mode        (const px4_msgs::msg::VehicleControlMode::SharedPtr        msg){ rpb_vehicle_control_mode        ->publish(*msg); }
    void cb_vehicle_global_position     (const px4_msgs::msg::VehicleGlobalPosition::SharedPtr     msg){ rpb_vehicle_global_position     ->publish(*msg); px4_geo_pos = *msg; px4_geo_pos_good = true; }
    void cb_vehicle_gps_position        (const px4_msgs::msg::SensorGps::SharedPtr                 msg){ rpb_vehicle_gps_position        ->publish(*msg); }
    void cb_vehicle_land_detected       (const px4_msgs::msg::VehicleLandDetected::SharedPtr       msg){ rpb_vehicle_land_detected       ->publish(*msg); }
    void cb_vehicle_local_position      (const px4_msgs::msg::VehicleLocalPosition::SharedPtr      msg){ rpb_vehicle_local_position      ->publish(*msg); px4_cpos = *msg; px4_cpos_good = true; }
    void cb_vehicle_odometry            (const px4_msgs::msg::VehicleOdometry::SharedPtr           msg){ rpb_vehicle_odometry            ->publish(*msg); grab_odom(msg); }
    void cb_vehicle_status_v1           (const px4_msgs::msg::VehicleStatus::SharedPtr             msg){ rpb_vehicle_status_v1           ->publish(*msg); px4_status = *msg; px4_status_good = true; }
    void cb_vtol_vehicle_status         (const px4_msgs::msg::VtolVehicleStatus::SharedPtr         msg){ rpb_vtol_vehicle_status         ->publish(*msg); }
    
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
    void init_fmu_sub_pub();
    
    int spin_main();
    
    void grab_orientation(px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void calc_utm_tf(px4_msgs::msg::HomePosition::SharedPtr msg);
    void grab_odom(px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    
    int t_state;
};
