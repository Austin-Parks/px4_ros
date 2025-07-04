/* Author: Austin Parks
 *   Date:     07-02-2025
 * Debug republisher node: This node will subscribe to all natively advertised 
 * internal PX4 Firmware topics and republish them under a different namespace
 * using default QoS settings for rqt Topic Monitor plugin compatibility.
 */

//#include <stdio.h>
#include <stdint.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

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

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using namespace std::chrono;

class Px4RosNode : public rclcpp::Node
{
public:
    Px4RosNode();
    
    void arm();
    void disarm();
    
private:
    
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated        >::SharedPtr sub_airspeed_validated;
    rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest       >::SharedPtr sub_arming_check_request;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus            >::SharedPtr sub_battery_status;
    rclcpp::Subscription<px4_msgs::msg::CollisionConstraints     >::SharedPtr sub_collision_constraints;
    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags     >::SharedPtr sub_estimator_status_flags;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags            >::SharedPtr sub_failsafe_flags;
    rclcpp::Subscription<px4_msgs::msg::HomePosition             >::SharedPtr sub_home_position;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint    >::SharedPtr sub_manual_control_setpoint;
    rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse    >::SharedPtr sub_message_format_response;
    rclcpp::Subscription<px4_msgs::msg::ModeCompleted            >::SharedPtr sub_mode_completed;
    rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet  >::SharedPtr sub_position_setpoint_triplet;
    rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply>::SharedPtr sub_register_ext_component_reply;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined           >::SharedPtr sub_sensor_combined;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus           >::SharedPtr sub_timesync_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude          >::SharedPtr sub_vehicle_attitude;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck        >::SharedPtr sub_vehicle_command_ack;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode       >::SharedPtr sub_vehicle_control_mode;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition    >::SharedPtr sub_vehicle_global_position;
    rclcpp::Subscription<px4_msgs::msg::SensorGps                >::SharedPtr sub_vehicle_gps_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected      >::SharedPtr sub_vehicle_land_detected;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition     >::SharedPtr sub_vehicle_local_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry          >::SharedPtr sub_vehicle_odometry;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus            >::SharedPtr sub_vehicle_status_v1;
    rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus        >::SharedPtr sub_vtol_vehicle_status;
    
    rclcpp::Publisher<px4_msgs::msg::AirspeedValidated           >::SharedPtr pub_airspeed_validated;
    rclcpp::Publisher<px4_msgs::msg::ArmingCheckRequest          >::SharedPtr pub_arming_check_request;
    rclcpp::Publisher<px4_msgs::msg::BatteryStatus               >::SharedPtr pub_battery_status;
    rclcpp::Publisher<px4_msgs::msg::CollisionConstraints        >::SharedPtr pub_collision_constraints;
    rclcpp::Publisher<px4_msgs::msg::EstimatorStatusFlags        >::SharedPtr pub_estimator_status_flags;
    rclcpp::Publisher<px4_msgs::msg::FailsafeFlags               >::SharedPtr pub_failsafe_flags;
    rclcpp::Publisher<px4_msgs::msg::HomePosition                >::SharedPtr pub_home_position;
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint       >::SharedPtr pub_manual_control_setpoint;
    rclcpp::Publisher<px4_msgs::msg::MessageFormatResponse       >::SharedPtr pub_message_format_response;
    rclcpp::Publisher<px4_msgs::msg::ModeCompleted               >::SharedPtr pub_mode_completed;
    rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet     >::SharedPtr pub_position_setpoint_triplet;
    rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentReply   >::SharedPtr pub_register_ext_component_reply;
    rclcpp::Publisher<px4_msgs::msg::SensorCombined              >::SharedPtr pub_sensor_combined;
    rclcpp::Publisher<px4_msgs::msg::TimesyncStatus              >::SharedPtr pub_timesync_status;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude             >::SharedPtr pub_vehicle_attitude;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck           >::SharedPtr pub_vehicle_command_ack;
    rclcpp::Publisher<px4_msgs::msg::VehicleControlMode          >::SharedPtr pub_vehicle_control_mode;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition       >::SharedPtr pub_vehicle_global_position;
    rclcpp::Publisher<px4_msgs::msg::SensorGps                   >::SharedPtr pub_vehicle_gps_position;
    rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected         >::SharedPtr pub_vehicle_land_detected;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition        >::SharedPtr pub_vehicle_local_position;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry             >::SharedPtr pub_vehicle_odometry;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus               >::SharedPtr pub_vehicle_status_v1;
    rclcpp::Publisher<px4_msgs::msg::VtolVehicleStatus           >::SharedPtr pub_vtol_vehicle_status;
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode         >::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint          >::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand              >::SharedPtr vehicle_command_publisher_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped            >::SharedPtr pub_gm_ps_cpos_ned;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped            >::SharedPtr pub_gm_ps_dbg;
    
    int spin_cnt;
    px4_msgs::msg::HomePosition          px4_home_pos;
    px4_msgs::msg::VehicleLocalPosition  px4_cpos;
    px4_msgs::msg::VehicleAttitude       px4_catt;
    px4_msgs::msg::VehicleGlobalPosition px4_geo_pos;
    px4_msgs::msg::VehicleStatus         px4_status;
    
    bool px4_home_pos_good;
    bool px4_cpos_good;
    bool px4_catt_good;
    bool px4_geo_pos_good;
    bool px4_status_good;
    
    geometry_msgs::msg::PoseStamped gm_ps_cpos_enu;
    geometry_msgs::msg::PoseStamped gm_ps_dbg;
    
    geographic_msgs::msg::GeoPoint gp_home_pos;
    
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    bool ctrl_en;
    bool ctrl_pos;
    bool ctrl_vel;
    //rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    geometry_msgs::msg::TransformStamped tf_local_utm;
    geometry_msgs::msg::TransformStamped tf_fmu;
    geometry_msgs::msg::TransformStamped tf_base_link;
    bool tf_good;
    
    void cb_airspeed_validated          (const px4_msgs::msg::AirspeedValidated::SharedPtr         msg){ pub_airspeed_validated          ->publish(*msg); }
    void cb_arming_check_request        (const px4_msgs::msg::ArmingCheckRequest::SharedPtr        msg){ pub_arming_check_request        ->publish(*msg); }
    void cb_battery_status              (const px4_msgs::msg::BatteryStatus::SharedPtr             msg){ pub_battery_status              ->publish(*msg); }
    void cb_collision_constraints       (const px4_msgs::msg::CollisionConstraints::SharedPtr      msg){ pub_collision_constraints       ->publish(*msg); }
    void cb_estimator_status_flags      (const px4_msgs::msg::EstimatorStatusFlags::SharedPtr      msg){ pub_estimator_status_flags      ->publish(*msg); }
    void cb_failsafe_flags              (const px4_msgs::msg::FailsafeFlags::SharedPtr             msg){ pub_failsafe_flags              ->publish(*msg); }
    void cb_home_position               (const px4_msgs::msg::HomePosition::SharedPtr              msg){ pub_home_position               ->publish(*msg); if(!px4_home_pos_good && px4_cpos_good && px4_catt_good) calc_tf(msg); }
    void cb_manual_control_setpoint     (const px4_msgs::msg::ManualControlSetpoint::SharedPtr     msg){ pub_manual_control_setpoint     ->publish(*msg); }
    void cb_message_format_response     (const px4_msgs::msg::MessageFormatResponse::SharedPtr     msg){ pub_message_format_response     ->publish(*msg); }
    void cb_mode_completed              (const px4_msgs::msg::ModeCompleted::SharedPtr             msg){ pub_mode_completed              ->publish(*msg); }
    void cb_position_setpoint_triplet   (const px4_msgs::msg::PositionSetpointTriplet::SharedPtr   msg){ pub_position_setpoint_triplet   ->publish(*msg); }
    void cb_register_ext_component_reply(const px4_msgs::msg::RegisterExtComponentReply::SharedPtr msg){ pub_register_ext_component_reply->publish(*msg); }
    void cb_sensor_combined             (const px4_msgs::msg::SensorCombined::SharedPtr            msg){ pub_sensor_combined             ->publish(*msg); }
    void cb_timesync_status             (const px4_msgs::msg::TimesyncStatus::SharedPtr            msg){ pub_timesync_status             ->publish(*msg); }
    void cb_vehicle_attitude            (const px4_msgs::msg::VehicleAttitude::SharedPtr           msg){ pub_vehicle_attitude            ->publish(*msg); grab_orientation(msg); }
    void cb_vehicle_command_ack         (const px4_msgs::msg::VehicleCommandAck::SharedPtr         msg){ pub_vehicle_command_ack         ->publish(*msg); }
    void cb_vehicle_control_mode        (const px4_msgs::msg::VehicleControlMode::SharedPtr        msg){ pub_vehicle_control_mode        ->publish(*msg); }
    void cb_vehicle_global_position     (const px4_msgs::msg::VehicleGlobalPosition::SharedPtr     msg){ pub_vehicle_global_position     ->publish(*msg); px4_geo_pos = *msg; px4_geo_pos_good = true; }
    void cb_vehicle_gps_position        (const px4_msgs::msg::SensorGps::SharedPtr                 msg){ pub_vehicle_gps_position        ->publish(*msg); }
    void cb_vehicle_land_detected       (const px4_msgs::msg::VehicleLandDetected::SharedPtr       msg){ pub_vehicle_land_detected       ->publish(*msg); }
    void cb_vehicle_local_position      (const px4_msgs::msg::VehicleLocalPosition::SharedPtr      msg){ pub_vehicle_local_position      ->publish(*msg); publish_tf(msg); }
    void cb_vehicle_odometry            (const px4_msgs::msg::VehicleOdometry::SharedPtr           msg){ pub_vehicle_odometry            ->publish(*msg); }
    void cb_vehicle_status_v1           (const px4_msgs::msg::VehicleStatus::SharedPtr             msg){ pub_vehicle_status_v1           ->publish(*msg); px4_status = *msg; px4_status_good = true; }
    void cb_vtol_vehicle_status         (const px4_msgs::msg::VtolVehicleStatus::SharedPtr         msg){ pub_vtol_vehicle_status         ->publish(*msg); }
    
    /*
    auto timer_callback = [this]() -> void {
        
        if (offboard_setpoint_counter_ == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            
            // Arm the vehicle
            this->arm();
        }
        
        // offboard_control_mode needs to be paired with trajectory_setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        
        // stop the counter after reaching 11
        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    };//*/
    
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
    void init_fmu_sub_pub();
    
    int spin_main();
    
    void grab_orientation(px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void calc_tf(px4_msgs::msg::HomePosition::SharedPtr msg);
    void publish_tf(px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    
    int t_state;
};
