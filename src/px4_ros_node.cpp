/* Author: Austin Parks
 *   Date:     07-02-2025
 * Debug republisher node: This node will subscribe to all natively advertised 
 * internal PX4 Firmware topics and republish them under a different namespace
 * using default QoS settings for rqt Topic Monitor plugin compatibility.
 */
#include "px4_ros_node.hpp"

using namespace std::chrono;


Px4RosNode::Px4RosNode() : Node("px4_ros_node")
{
    px4_home_pos_good = false;
    px4_cpos_good = false;
    px4_catt_good = false;
    px4_geo_pos_good = false;
    tf_good = false;
    
    ctrl_pos = false;
    ctrl_vel = true;
    t_state = 0;
    
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
    
    pub_airspeed_validated           = this->create_publisher<px4_msgs::msg::AirspeedValidated        >("/dbg/fmu/out/airspeed_validated_v1",        10);
    pub_arming_check_request         = this->create_publisher<px4_msgs::msg::ArmingCheckRequest       >("/dbg/fmu/out/arming_check_request",         10);
    pub_battery_status               = this->create_publisher<px4_msgs::msg::BatteryStatus            >("/dbg/fmu/out/battery_status",               10);
    pub_collision_constraints        = this->create_publisher<px4_msgs::msg::CollisionConstraints     >("/dbg/fmu/out/collision_constraints",        10);
    pub_estimator_status_flags       = this->create_publisher<px4_msgs::msg::EstimatorStatusFlags     >("/dbg/fmu/out/estimator_status_flags",       10);
    pub_failsafe_flags               = this->create_publisher<px4_msgs::msg::FailsafeFlags            >("/dbg/fmu/out/failsafe_flags",               10);
    pub_home_position                = this->create_publisher<px4_msgs::msg::HomePosition             >("/dbg/fmu/out/home_position",                10);
    pub_manual_control_setpoint      = this->create_publisher<px4_msgs::msg::ManualControlSetpoint    >("/dbg/fmu/out/manual_control_setpoint",      10);
    pub_message_format_response      = this->create_publisher<px4_msgs::msg::MessageFormatResponse    >("/dbg/fmu/out/message_format_response",      10);
    pub_mode_completed               = this->create_publisher<px4_msgs::msg::ModeCompleted            >("/dbg/fmu/out/mode_completed",               10);
    pub_position_setpoint_triplet    = this->create_publisher<px4_msgs::msg::PositionSetpointTriplet  >("/dbg/fmu/out/position_setpoint_triplet",    10);
    pub_register_ext_component_reply = this->create_publisher<px4_msgs::msg::RegisterExtComponentReply>("/dbg/fmu/out/register_ext_component_reply", 10);
    pub_sensor_combined              = this->create_publisher<px4_msgs::msg::SensorCombined           >("/dbg/fmu/out/sensor_combined",              10);
    pub_timesync_status              = this->create_publisher<px4_msgs::msg::TimesyncStatus           >("/dbg/fmu/out/timesync_status",              10);
    pub_vehicle_attitude             = this->create_publisher<px4_msgs::msg::VehicleAttitude          >("/dbg/fmu/out/vehicle_attitude",             10);
    pub_vehicle_command_ack          = this->create_publisher<px4_msgs::msg::VehicleCommandAck        >("/dbg/fmu/out/vehicle_command_ack",          10);
    pub_vehicle_control_mode         = this->create_publisher<px4_msgs::msg::VehicleControlMode       >("/dbg/fmu/out/vehicle_control_mode",         10);
    pub_vehicle_global_position      = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition    >("/dbg/fmu/out/vehicle_global_position",      10);
    pub_vehicle_gps_position         = this->create_publisher<px4_msgs::msg::SensorGps                >("/dbg/fmu/out/vehicle_gps_position",         10);
    pub_vehicle_land_detected        = this->create_publisher<px4_msgs::msg::VehicleLandDetected      >("/dbg/fmu/out/vehicle_land_detected",        10);
    pub_vehicle_local_position       = this->create_publisher<px4_msgs::msg::VehicleLocalPosition     >("/dbg/fmu/out/vehicle_local_position",       10);
    pub_vehicle_odometry             = this->create_publisher<px4_msgs::msg::VehicleOdometry          >("/dbg/fmu/out/vehicle_odometry",             10);
    pub_vehicle_status_v1            = this->create_publisher<px4_msgs::msg::VehicleStatus            >("/dbg/fmu/out/vehicle_status_v1",            10);
    pub_vtol_vehicle_status          = this->create_publisher<px4_msgs::msg::VtolVehicleStatus        >("/dbg/fmu/out/vtol_vehicle_status",          10);
    
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode      >("/fmu/in/offboard_control_mode",             10);
    trajectory_setpoint_publisher_   = this->create_publisher<px4_msgs::msg::TrajectorySetpoint       >("/fmu/in/trajectory_setpoint",               10);
    vehicle_command_publisher_       = this->create_publisher<px4_msgs::msg::VehicleCommand           >("/fmu/in/vehicle_command",                   10);
    
    pub_gm_ps_cpos_ned               = this->create_publisher<geometry_msgs::msg::PoseStamped         >("/px4ros/out/vehicle_local_position",        10);
    
    tf_local_utm.header.frame_id   = "utm_origin";
    tf_local_utm.child_frame_id    = "local_enu_origin";
    
    tf_fmu.header.frame_id         = "local_enu_origin";
    tf_fmu.child_frame_id          = "fmu_origin";
    
    gm_ps_cpos_enu.header.frame_id = "local_enu_origin";
    
    offboard_setpoint_counter_ = 0;
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
    };
    
    timer_ = this->create_wall_timer(100ms, timer_callback);
    
    // Initialize the transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Px4RosNode::calc_tf(px4_msgs::msg::HomePosition::SharedPtr msg)
{
    px4_home_pos = *msg;
    px4_home_pos_good = true;
    gp_home_pos.altitude  = msg->alt;
    gp_home_pos.latitude  = msg->lat;
    gp_home_pos.longitude = msg->lon;
    
    tf2::Quaternion ori;
    geodesy::UTMPoint utm_home_pos;
    geodesy::fromMsg(gp_home_pos, utm_home_pos);
    
    ori.setRPY(0.0, 0.0, 0.0);
    tf_local_utm.transform.translation.x =  utm_home_pos.easting;
    tf_local_utm.transform.translation.y =  utm_home_pos.northing;
    tf_local_utm.transform.translation.z =  utm_home_pos.altitude;
    tf_local_utm.transform.rotation.w    = ori.w();
    tf_local_utm.transform.rotation.x    = ori.x();
    tf_local_utm.transform.rotation.y    = ori.y();
    tf_local_utm.transform.rotation.z    = ori.z();
    tf_good = true;
    printf("\nHome Position Aquired - Transform now available...\n\n");
}

void Px4RosNode::grab_orientation(px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    px4_catt = *msg;
    px4_catt_good = true;
    
    // According to https://docs.px4.io/main/en/msg_docs/VehicleAttitude.html
    // The order is orientation(w, x, y, z)
    gm_ps_cpos_enu.header.stamp = this->get_clock()->now();
    gm_ps_cpos_enu.pose.orientation.w =   msg->q[0]; // W
    gm_ps_cpos_enu.pose.orientation.x =   msg->q[1]; // X
    gm_ps_cpos_enu.pose.orientation.y =   msg->q[2]; // Y
    gm_ps_cpos_enu.pose.orientation.z =  (-1.0 * (double)(msg->q[3])); // Z
    tf_fmu.transform.rotation.w    = gm_ps_cpos_enu.pose.orientation.w;
    tf_fmu.transform.rotation.x    = gm_ps_cpos_enu.pose.orientation.x;
    tf_fmu.transform.rotation.y    = gm_ps_cpos_enu.pose.orientation.y;
    tf_fmu.transform.rotation.z    = gm_ps_cpos_enu.pose.orientation.z;
}

void Px4RosNode::publish_tf(px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    px4_cpos = *msg;
    px4_cpos_good = true;
    // Publish ENU Pose
    gm_ps_cpos_enu.header.stamp = this->get_clock()->now();
    gm_ps_cpos_enu.pose.position.x =  msg->y;
    gm_ps_cpos_enu.pose.position.y =  msg->x;
    gm_ps_cpos_enu.pose.position.z = -msg->z;
    pub_gm_ps_cpos_ned->publish(gm_ps_cpos_enu);
    
    if(tf_good)
    {
        tf_local_utm.header.stamp = gm_ps_cpos_enu.header.stamp;
        tf_fmu.header.stamp = gm_ps_cpos_enu.header.stamp;
        
        tf_fmu.transform.translation.x =  gm_ps_cpos_enu.pose.position.x;
        tf_fmu.transform.translation.y =  gm_ps_cpos_enu.pose.position.y;
        tf_fmu.transform.translation.z =  gm_ps_cpos_enu.pose.position.z;
        
        tf_broadcaster->sendTransform(tf_local_utm);
        tf_broadcaster->sendTransform(tf_fmu);
    }
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
    vehicle_command_publisher_->publish(msg);
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
    offboard_control_mode_publisher_->publish(msg);
}

/** @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees. */
void Px4RosNode::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    ctrl_pos = false;
    ctrl_vel = true;
    
    if(t_state < 2)
    {
        if(px4_cpos.z > -10.0)
        {   // NED Coordinates
            msg.position = {NAN, NAN, NAN};
            msg.velocity = {0.0, 0.0, -1.5};
        }
        else
        {
            // NED Coordinates
            if(px4_cpos.y < 10.0 && t_state < 1) {
                msg.position = {NAN, NAN, NAN};
                msg.velocity = {0.0,  1.0, 0.0}; }
            else {
                t_state = 1;
                if(px4_cpos.y >= 0.0) {
                    msg.position = {NAN, NAN, NAN};
                    msg.velocity = {0.0, -1.0, 0.0}; }
                else
                    t_state = 2;
            }
        }
    }
    else
    {
        if(gm_ps_cpos_enu.pose.position.z > 0.05)
        {
            ctrl_pos = true;
            ctrl_vel = false;
            publish_offboard_control_mode();
            msg.position = {0.0, 0.0, 0.0};
            msg.velocity = {NAN, NAN, NAN};
        }
        else
        {
            disarm();
        }
            
    }
    
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4RosNode>());
    rclcpp::shutdown();
    return 0;
}
