/* Author: Austin Parks
 *   Date:     07-02-2025
 * Debug republisher node: This node will subscribe to all natively advertised 
 * internal PX4 Firmware topics and republish them under a different namespace
 * using default QoS settings for rqt Topic Monitor plugin compatibility.
 */
#include "px4_ros_node.hpp"

using namespace std::chrono;

void Px4RosNode::init_fmu_sub_pub()
{
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    
    sub_airspeed_validated           = this->create_subscription<px4_msgs::msg::AirspeedValidated        >("/fmu/out/airspeed_validated_v1",        qos, std::bind(&Px4RosNode::cb_airspeed_validated          , this, std::placeholders::_1));
    sub_arming_check_request         = this->create_subscription<px4_msgs::msg::ArmingCheckRequest       >("/fmu/out/arming_check_request",         qos, std::bind(&Px4RosNode::cb_arming_check_request        , this, std::placeholders::_1));
    sub_battery_status               = this->create_subscription<px4_msgs::msg::BatteryStatus            >("/fmu/out/battery_status",               qos, std::bind(&Px4RosNode::cb_battery_status              , this, std::placeholders::_1));
    sub_collision_constraints        = this->create_subscription<px4_msgs::msg::CollisionConstraints     >("/fmu/out/collision_constraints",        qos, std::bind(&Px4RosNode::cb_collision_constraints       , this, std::placeholders::_1));
    sub_estimator_status_flags       = this->create_subscription<px4_msgs::msg::EstimatorStatusFlags     >("/fmu/out/estimator_status_flags",       qos, std::bind(&Px4RosNode::cb_estimator_status_flags      , this, std::placeholders::_1));
    sub_failsafe_flags               = this->create_subscription<px4_msgs::msg::FailsafeFlags            >("/fmu/out/failsafe_flags",               qos, std::bind(&Px4RosNode::cb_failsafe_flags              , this, std::placeholders::_1));
    sub_home_position                = this->create_subscription<px4_msgs::msg::HomePosition             >("/fmu/out/home_position",                qos, std::bind(&Px4RosNode::cb_home_position               , this, std::placeholders::_1));
    sub_manual_control_setpoint      = this->create_subscription<px4_msgs::msg::ManualControlSetpoint    >("/fmu/out/manual_control_setpoint",      qos, std::bind(&Px4RosNode::cb_manual_control_setpoint     , this, std::placeholders::_1));
    sub_message_format_response      = this->create_subscription<px4_msgs::msg::MessageFormatResponse    >("/fmu/out/message_format_response",      qos, std::bind(&Px4RosNode::cb_message_format_response     , this, std::placeholders::_1));
    sub_mode_completed               = this->create_subscription<px4_msgs::msg::ModeCompleted            >("/fmu/out/mode_completed",               qos, std::bind(&Px4RosNode::cb_mode_completed              , this, std::placeholders::_1));
    sub_position_setpoint_triplet    = this->create_subscription<px4_msgs::msg::PositionSetpointTriplet  >("/fmu/out/position_setpoint_triplet",    qos, std::bind(&Px4RosNode::cb_position_setpoint_triplet   , this, std::placeholders::_1));
    sub_register_ext_component_reply = this->create_subscription<px4_msgs::msg::RegisterExtComponentReply>("/fmu/out/register_ext_component_reply", qos, std::bind(&Px4RosNode::cb_register_ext_component_reply, this, std::placeholders::_1));
    sub_sensor_combined              = this->create_subscription<px4_msgs::msg::SensorCombined           >("/fmu/out/sensor_combined",              qos, std::bind(&Px4RosNode::cb_sensor_combined             , this, std::placeholders::_1));
    sub_timesync_status              = this->create_subscription<px4_msgs::msg::TimesyncStatus           >("/fmu/out/timesync_status",              qos, std::bind(&Px4RosNode::cb_timesync_status             , this, std::placeholders::_1));
    sub_vehicle_attitude             = this->create_subscription<px4_msgs::msg::VehicleAttitude          >("/fmu/out/vehicle_attitude",             qos, std::bind(&Px4RosNode::cb_vehicle_attitude            , this, std::placeholders::_1));
    sub_vehicle_command_ack          = this->create_subscription<px4_msgs::msg::VehicleCommandAck        >("/fmu/out/vehicle_command_ack",          qos, std::bind(&Px4RosNode::cb_vehicle_command_ack         , this, std::placeholders::_1));
    sub_vehicle_control_mode         = this->create_subscription<px4_msgs::msg::VehicleControlMode       >("/fmu/out/vehicle_control_mode",         qos, std::bind(&Px4RosNode::cb_vehicle_control_mode        , this, std::placeholders::_1));
    sub_vehicle_global_position      = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition    >("/fmu/out/vehicle_global_position",      qos, std::bind(&Px4RosNode::cb_vehicle_global_position     , this, std::placeholders::_1));
    sub_vehicle_gps_position         = this->create_subscription<px4_msgs::msg::SensorGps                >("/fmu/out/vehicle_gps_position",         qos, std::bind(&Px4RosNode::cb_vehicle_gps_position        , this, std::placeholders::_1));
    sub_vehicle_land_detected        = this->create_subscription<px4_msgs::msg::VehicleLandDetected      >("/fmu/out/vehicle_land_detected",        qos, std::bind(&Px4RosNode::cb_vehicle_land_detected       , this, std::placeholders::_1));
    sub_vehicle_local_position       = this->create_subscription<px4_msgs::msg::VehicleLocalPosition     >("/fmu/out/vehicle_local_position",       qos, std::bind(&Px4RosNode::cb_vehicle_local_position      , this, std::placeholders::_1));
    sub_vehicle_odometry             = this->create_subscription<px4_msgs::msg::VehicleOdometry          >("/fmu/out/vehicle_odometry",             qos, std::bind(&Px4RosNode::cb_vehicle_odometry            , this, std::placeholders::_1));
    sub_vehicle_status_v1            = this->create_subscription<px4_msgs::msg::VehicleStatus            >("/fmu/out/vehicle_status_v1",            qos, std::bind(&Px4RosNode::cb_vehicle_status_v1           , this, std::placeholders::_1));
    sub_vtol_vehicle_status          = this->create_subscription<px4_msgs::msg::VtolVehicleStatus        >("/fmu/out/vtol_vehicle_status",          qos, std::bind(&Px4RosNode::cb_vtol_vehicle_status         , this, std::placeholders::_1));
    
    rpb_airspeed_validated           = this->create_publisher<px4_msgs::msg::AirspeedValidated        >("/px4_ros/fmu/out/airspeed_validated_v1",        10);
    rpb_arming_check_request         = this->create_publisher<px4_msgs::msg::ArmingCheckRequest       >("/px4_ros/fmu/out/arming_check_request",         10);
    rpb_battery_status               = this->create_publisher<px4_msgs::msg::BatteryStatus            >("/px4_ros/fmu/out/battery_status",               10);
    rpb_collision_constraints        = this->create_publisher<px4_msgs::msg::CollisionConstraints     >("/px4_ros/fmu/out/collision_constraints",        10);
    rpb_estimator_status_flags       = this->create_publisher<px4_msgs::msg::EstimatorStatusFlags     >("/px4_ros/fmu/out/estimator_status_flags",       10);
    rpb_failsafe_flags               = this->create_publisher<px4_msgs::msg::FailsafeFlags            >("/px4_ros/fmu/out/failsafe_flags",               10);
    rpb_home_position                = this->create_publisher<px4_msgs::msg::HomePosition             >("/px4_ros/fmu/out/home_position",                10);
    rpb_manual_control_setpoint      = this->create_publisher<px4_msgs::msg::ManualControlSetpoint    >("/px4_ros/fmu/out/manual_control_setpoint",      10);
    rpb_message_format_response      = this->create_publisher<px4_msgs::msg::MessageFormatResponse    >("/px4_ros/fmu/out/message_format_response",      10);
    rpb_mode_completed               = this->create_publisher<px4_msgs::msg::ModeCompleted            >("/px4_ros/fmu/out/mode_completed",               10);
    rpb_position_setpoint_triplet    = this->create_publisher<px4_msgs::msg::PositionSetpointTriplet  >("/px4_ros/fmu/out/position_setpoint_triplet",    10);
    rpb_register_ext_component_reply = this->create_publisher<px4_msgs::msg::RegisterExtComponentReply>("/px4_ros/fmu/out/register_ext_component_reply", 10);
    rpb_sensor_combined              = this->create_publisher<px4_msgs::msg::SensorCombined           >("/px4_ros/fmu/out/sensor_combined",              10);
    rpb_timesync_status              = this->create_publisher<px4_msgs::msg::TimesyncStatus           >("/px4_ros/fmu/out/timesync_status",              10);
    rpb_vehicle_attitude             = this->create_publisher<px4_msgs::msg::VehicleAttitude          >("/px4_ros/fmu/out/vehicle_attitude",             10);
    rpb_vehicle_command_ack          = this->create_publisher<px4_msgs::msg::VehicleCommandAck        >("/px4_ros/fmu/out/vehicle_command_ack",          10);
    rpb_vehicle_control_mode         = this->create_publisher<px4_msgs::msg::VehicleControlMode       >("/px4_ros/fmu/out/vehicle_control_mode",         10);
    rpb_vehicle_global_position      = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition    >("/px4_ros/fmu/out/vehicle_global_position",      10);
    rpb_vehicle_gps_position         = this->create_publisher<px4_msgs::msg::SensorGps                >("/px4_ros/fmu/out/vehicle_gps_position",         10);
    rpb_vehicle_land_detected        = this->create_publisher<px4_msgs::msg::VehicleLandDetected      >("/px4_ros/fmu/out/vehicle_land_detected",        10);
    rpb_vehicle_local_position       = this->create_publisher<px4_msgs::msg::VehicleLocalPosition     >("/px4_ros/fmu/out/vehicle_local_position",       10);
    rpb_vehicle_odometry             = this->create_publisher<px4_msgs::msg::VehicleOdometry          >("/px4_ros/fmu/out/vehicle_odometry",             10);
    rpb_vehicle_status_v1            = this->create_publisher<px4_msgs::msg::VehicleStatus            >("/px4_ros/fmu/out/vehicle_status_v1",            10);
    rpb_vtol_vehicle_status          = this->create_publisher<px4_msgs::msg::VtolVehicleStatus        >("/px4_ros/fmu/out/vtol_vehicle_status",          10);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pub_actuator_motors                    = this->create_publisher<px4_msgs::msg::ActuatorMotors                  >("/fmu/in/actuator_motors",                    qos);
	pub_actuator_servos                    = this->create_publisher<px4_msgs::msg::ActuatorServos                  >("/fmu/in/actuator_servos",                    qos);
	pub_arming_check_reply_v1              = this->create_publisher<px4_msgs::msg::ArmingCheckReply                >("/fmu/in/arming_check_reply_v1",              qos);
	pub_aux_global_position                = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition           >("/fmu/in/aux_global_position",                qos);
	pub_config_control_setpoints           = this->create_publisher<px4_msgs::msg::VehicleControlMode              >("/fmu/in/config_control_setpoints",           qos);
	pub_config_overrides_request           = this->create_publisher<px4_msgs::msg::ConfigOverrides                 >("/fmu/in/config_overrides_request",           qos);
	pub_distance_sensor                    = this->create_publisher<px4_msgs::msg::DistanceSensor                  >("/fmu/in/distance_sensor",                    qos);
	pub_fixed_wing_lateral_setpoint        = this->create_publisher<px4_msgs::msg::FixedWingLateralSetpoint        >("/fmu/in/fixed_wing_lateral_setpoint",        qos);
	pub_fixed_wing_longitudinal_setpoint   = this->create_publisher<px4_msgs::msg::FixedWingLongitudinalSetpoint   >("/fmu/in/fixed_wing_longitudinal_setpoint",   qos);
	pub_goto_setpoint                      = this->create_publisher<px4_msgs::msg::GotoSetpoint                    >("/fmu/in/goto_setpoint",                      qos);
	pub_lateral_control_configuration      = this->create_publisher<px4_msgs::msg::LateralControlConfiguration     >("/fmu/in/lateral_control_configuration",      qos);
	pub_longitudinal_control_configuration = this->create_publisher<px4_msgs::msg::LongitudinalControlConfiguration>("/fmu/in/longitudinal_control_configuration", qos);
	pub_manual_control_input               = this->create_publisher<px4_msgs::msg::ManualControlSetpoint           >("/fmu/in/manual_control_input",               qos);
	pub_message_format_request             = this->create_publisher<px4_msgs::msg::MessageFormatRequest            >("/fmu/in/message_format_request",             qos);
	pub_mode_completed                     = this->create_publisher<px4_msgs::msg::ModeCompleted                   >("/fmu/in/mode_completed",                     qos);
	pub_obstacle_distance                  = this->create_publisher<px4_msgs::msg::ObstacleDistance                >("/fmu/in/obstacle_distance",                  qos);
	pub_offboard_control_mode              = this->create_publisher<px4_msgs::msg::OffboardControlMode             >("/fmu/in/offboard_control_mode",              qos);
	pub_onboard_computer_status            = this->create_publisher<px4_msgs::msg::OnboardComputerStatus           >("/fmu/in/onboard_computer_status",            qos);
	pub_register_ext_component_request     = this->create_publisher<px4_msgs::msg::RegisterExtComponentRequest     >("/fmu/in/register_ext_component_request",     qos);
	pub_sensor_optical_flow                = this->create_publisher<px4_msgs::msg::SensorOpticalFlow               >("/fmu/in/sensor_optical_flow",                qos);
	pub_telemetry_status                   = this->create_publisher<px4_msgs::msg::TelemetryStatus                 >("/fmu/in/telemetry_status",                   qos);
	pub_trajectory_setpoint                = this->create_publisher<px4_msgs::msg::TrajectorySetpoint              >("/fmu/in/trajectory_setpoint",                qos);
	pub_unregister_ext_component           = this->create_publisher<px4_msgs::msg::UnregisterExtComponent          >("/fmu/in/unregister_ext_component",           qos);
	pub_vehicle_attitude_setpoint_v1       = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint         >("/fmu/in/vehicle_attitude_setpoint_v1",       qos);
	pub_vehicle_command                    = this->create_publisher<px4_msgs::msg::VehicleCommand                  >("/fmu/in/vehicle_command",                    qos);
	pub_vehicle_command_mode_executor      = this->create_publisher<px4_msgs::msg::VehicleCommand                  >("/fmu/in/vehicle_command_mode_executor",      qos);
	pub_vehicle_mocap_odometry             = this->create_publisher<px4_msgs::msg::VehicleOdometry                 >("/fmu/in/vehicle_mocap_odometry",             qos);
	pub_vehicle_rates_setpoint             = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint            >("/fmu/in/vehicle_rates_setpoint",             qos);
	pub_vehicle_thrust_setpoint            = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint           >("/fmu/in/vehicle_thrust_setpoint",            qos);
	pub_vehicle_torque_setpoint            = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint           >("/fmu/in/vehicle_torque_setpoint",            qos);
	pub_vehicle_visual_odometry            = this->create_publisher<px4_msgs::msg::VehicleOdometry                 >("/fmu/in/vehicle_visual_odometry",            qos);
    
}

Px4RosNode::Px4RosNode() : Node("px4_ros_node")
{
    spin_cnt = 0;
    px4_odom_good = false;
    px4_home_pos_good = false;
    px4_cpos_good = false;
    px4_catt_good = false;
    px4_geo_pos_good = false;
    tf_good = false;
    
    ctrl_en  = true;
    ctrl_pos = false;
    ctrl_vel = true;
    t_state = 0;
    
    init_fmu_sub_pub();
    
    pub_ros_pose_enu               = this->create_publisher<geometry_msgs::msg::PoseStamped>("/px4ros/out/vehicle_local_position",        10);
    pub_ros_odom                   = this->create_publisher<nav_msgs::msg::Odometry        >("/px4ros/out/odom",                          10);
    pub_ros_dbg_pose               = this->create_publisher<geometry_msgs::msg::PoseStamped>("/px4ros/out/dbg_pose",                      10);
    
    tf_local_utm.header.frame_id   = "utm_origin";
    tf_local_utm.child_frame_id    = "local_enu_origin";
    
    tf_fmu.header.frame_id         = "local_enu_origin";
    tf_fmu.child_frame_id          = "base_link";
    
    //tf_base_link.header.frame_id   = "fmu_origin";
    //tf_base_link.child_frame_id    = "base_link";
    
    ros_odom.header.frame_id       = "local_enu_origin";
    ros_odom.child_frame_id        = "base_link";
    
    ros_pose_enu.header.frame_id   = "base_link";
    ros_dbg_pose.header.frame_id   = "local_enu_origin";
    
    // Initialize the transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Debug point
    ros_dbg_pose.pose.position.x =  5.0;
    ros_dbg_pose.pose.position.y =  5.0;
    ros_dbg_pose.pose.position.z =  5.0;
    tf2::Quaternion ori;
    ori.setRPY(  0.0,  0.0,  3.141592654 );
    ros_dbg_pose.pose.orientation.w =  ori.w();
    ros_dbg_pose.pose.orientation.x =  ori.x();
    ros_dbg_pose.pose.orientation.y =  ori.y();
    ros_dbg_pose.pose.orientation.z =  ori.z();
    
    double theta_T = 30.0;
    theta_traj = 0.0;
    theta_traj_dx = (2 * 3.141592654) / (theta_T * 100.0);
    
    // run spin_main() at 100 Hz
    timer_ = this->create_wall_timer(10ms, std::bind(&Px4RosNode::spin_main, this) );
}

int Px4RosNode::spin_main()
{
    if (spin_cnt == 100) {
        // Change to Offboard mode after 50 setpoints
        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        
        // Arm the vehicle
        this->arm();
    }
    
    // offboard_control_mode needs to be paired with trajectory_setpoint
    if(ctrl_en)
    {
        publish_trajectory_setpoint();
        publish_offboard_control_mode();
    }
    
    ros_dbg_pose.header.stamp = ros_odom.header.stamp;
    //pub_gm_ps_dbg->publish(gm_ps_dbg);
    
    // stop the counter after reaching 11
    spin_cnt++;
    return 0;
}

void Px4RosNode::calc_utm_tf(px4_msgs::msg::HomePosition::SharedPtr msg)
{
    px4_home_pos = *msg;
    px4_home_pos_good = true;
    ros_home_pos.altitude  = msg->alt;
    ros_home_pos.latitude  = msg->lat;
    ros_home_pos.longitude = msg->lon;
    
    tf2::Quaternion ori;
    geodesy::UTMPoint utm_home_pos;
    geodesy::fromMsg(ros_home_pos, utm_home_pos);
    
    ori.setRPY(0.0, 0.0, 0.0 );
    tf_local_utm.transform.translation.x =  utm_home_pos.easting;
    tf_local_utm.transform.translation.y =  utm_home_pos.northing;
    tf_local_utm.transform.translation.z =  utm_home_pos.altitude;
    tf_local_utm.transform.rotation.w    = ori.w();
    tf_local_utm.transform.rotation.x    = ori.x();
    tf_local_utm.transform.rotation.y    = ori.y();
    tf_local_utm.transform.rotation.z    = ori.z();
    
    printf("\nHome Position Aquired - Transform now available...\n\n");
}

void Px4RosNode::grab_orientation(px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    // According to https://docs.px4.io/main/en/msg_docs/VehicleAttitude.html
    // The order is orientation(w, x, y, z)   
    px4_catt = *msg;
    px4_catt_good = true;
    ros_pose_enu.header.stamp = this->get_clock()->now();
    /*
    ori.setW(  1.0 * (double)(px4_catt.q[0]) ); //  W
    ori.setY(  1.0 * (double)(px4_catt.q[1]) ); //  X (Northing)
    ori.setX(  1.0 * (double)(px4_catt.q[2]) ); //  Y (Easting)
    ori.setZ( -1.0 * (double)(px4_catt.q[3]) ); //  Z (Upping)
    ori = ori.inverse();
    tf2::Matrix3x3 mat(ori);
    double roll, pitch, yaw;
    mat.getRPY( roll, pitch, yaw );
    //*/
}

void Px4RosNode::grab_odom(px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    px4_odom = *msg;
    px4_odom_good = true;
    tf2::Quaternion ori;
    
    ros_odom.header.stamp = this->get_clock()->now();
    
    tf_good = px4_home_pos_good & px4_odom_good;
    if(tf_good)
    {
        ////////////////////////////////////////////////////////////////////
        tf_local_utm.header.stamp = ros_odom.header.stamp;
        tf_fmu.header.stamp       = ros_odom.header.stamp;
        //tf_base_link.header.stamp = ros_odom.header.stamp;
        ////////////////////////////////////////////////////////////////////
        tf_fmu.transform.translation.y =  px4_odom.position[0];
        tf_fmu.transform.translation.x =  px4_odom.position[1];
        tf_fmu.transform.translation.z = -px4_odom.position[2];
        ori.setW(  1.0 * (double)(px4_odom.q[0]) );
        ori.setX(  1.0 * (double)(px4_odom.q[1]) );
        ori.setY(  1.0 * (double)(px4_odom.q[2]) );
        ori.setZ( -1.0 * (double)(px4_odom.q[3]) );
        tf2::Quaternion bl_ori;
        bl_ori.setRPY( 0.0, 0.0, 1.570796327); // + 1.570796327  3.141592654
        ori = ori * bl_ori;
        tf_fmu.transform.rotation.w    = ori.w();
        tf_fmu.transform.rotation.y    = ori.x();
        tf_fmu.transform.rotation.x    = ori.y();
        tf_fmu.transform.rotation.z    = ori.z();
        ////////////////////////////////////////////////////////////////////
        // tf_base_link.transform.translation.x = 0.0;
        // tf_base_link.transform.translation.y = 0.0;
        // tf_base_link.transform.translation.z = 0.0;
        // tf_base_link.transform.rotation.w    = bl_ori.w();
        // tf_base_link.transform.rotation.x    = bl_ori.x();
        // tf_base_link.transform.rotation.y    = bl_ori.y();
        // tf_base_link.transform.rotation.z    = bl_ori.z();
        ////////////////////////////////////////////////////////////////////
        tf_broadcaster->sendTransform(tf_local_utm);
        tf_broadcaster->sendTransform(tf_fmu);
        // tf_broadcaster->sendTransform(tf_base_link);
        ////////////////////////////////////////////////////////////////////
        ros_odom.pose.pose.position.x    = tf_fmu.transform.translation.x;
        ros_odom.pose.pose.position.y    = tf_fmu.transform.translation.y;
        ros_odom.pose.pose.position.z    = tf_fmu.transform.translation.z;
        ros_odom.pose.pose.orientation.w = ori.w();
        ros_odom.pose.pose.orientation.x = ori.x();
        ros_odom.pose.pose.orientation.y = ori.y();
        ros_odom.pose.pose.orientation.z = ori.z();
        pub_ros_odom->publish(ros_odom);
    }
    
    // Publish ENU Pose
    ros_pose_enu.header.stamp = ros_odom.header.stamp;
    ros_pose_enu.pose.position.x = 0; // fixed frame
    ros_pose_enu.pose.position.y = 0; // fixed frame
    ros_pose_enu.pose.position.z = 0; // fixed frame
    ori.setRPY( 0.0, 0.0, 0.0 );
    ros_pose_enu.pose.orientation.w = ori.w();
    ros_pose_enu.pose.orientation.x = ori.x();
    ros_pose_enu.pose.orientation.y = ori.y();
    ros_pose_enu.pose.orientation.z = ori.z();
    pub_ros_pose_enu->publish(ros_pose_enu);
}

/** @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2 */
void Px4RosNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_vehicle_command->publish(msg);
}
/** @brief Send a command to Arm the vehicle */
void Px4RosNode::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/** @brief Send a command to Disarm the vehicle */
void Px4RosNode::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/** @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active. */
void Px4RosNode::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = ctrl_pos;
    msg.velocity = ctrl_vel;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_offboard_control_mode->publish(msg);
}

/** @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees. */
void Px4RosNode::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    ctrl_pos = true;
    //ctrl_vel = true;
    
    if(t_state < 2)
    {
        if(px4_odom.position[2] > -5.0)
        {   // NED Coordinates
            msg.position = {0.0, 0.0, -5.0}; // {NAN, NAN, NAN};
            //msg.velocity = {0.0, 0.0, -3.0};
        }
        else
        {
            // NED Coordinates
            if( theta_traj < (2 * 3.141592654) ) {
                //msg.position = {NAN, NAN, NAN};
                //msg.velocity = {-15.0,  0.0, 0.0};
                theta_traj += theta_traj_dx;
                float east  =  5.0 * std::cos(1.0 * theta_traj);
                float north =  5.0 * std::sin(2.0 * theta_traj);
                msg.position = { north, east, NAN };
            }
            else {
                if(std::abs(px4_odom.position[0]) >= 0.01 || std::abs(px4_odom.position[1]) >= 0.01) {
                    ctrl_pos = true;
                    ctrl_vel = false;
                    msg.position = {0.0, 0.0, NAN};
                    ///msg.velocity = {NAN, NAN, 0.1};
                }
                else
                {
                    // ctrl_pos = false;
                    // ctrl_vel = true;
                    // //msg.position = {0.0, 0.0, 0.0};
                    // msg.velocity = {NAN, NAN, 0.1};
                    // if(px4_cpos.z > 0.05)
                    //     t_state = 3;
                    t_state = 3;
                }
            }
        }
    }
    else
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_PRECLAND, 1.0);
        if( px4_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED )
            ctrl_en = false;
    }
    
    msg.yaw = 0.0; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub_trajectory_setpoint->publish(msg);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4RosNode>());
    rclcpp::shutdown();
    return 0;
}
