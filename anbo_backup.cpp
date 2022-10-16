#include <dji_osdk_ros/anbo_node.h>
#include <time.h>

using namespace dji_osdk_ros;
  bool VehicleWrapper::pubLocalPose(const SetLocalPoseMsg& input) {
    if (!vehicle) {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    if (!startGlobalPositionBroadcast())
    {
      std::cout << "No Broadcast!" << std::endl;
      return false;
    }

    // get home point orientation
    tf::Quaternion q_home(homeQuaternion.q1,
                          homeQuaternion.q2,
                          homeQuaternion.q3,
                          homeQuaternion.q0);
    tf::Matrix3x3 R_home(q_home);
    tf::Vector3 input_local(input.x, input.y, input.z);
    tf::Vector3 input_TN(R_home * input_local);
    // get input x,y,z, yaw
    using namespace Telemetry;
    Vector3f offsetDesired_TN;
    offsetDesired_TN.x = input_TN.x();
    offsetDesired_TN.y = input_TN.y();
    offsetDesired_TN.z = input_TN.z();
    tf::Quaternion q_desired_local(input.q_x,
                                  input.q_y,
                                  input.q_z,
                                  input.q_w);
    tf::Matrix3x3 R_desired_TN(q_home * q_desired_local);
    double r_, p_, yaw_;
    R_desired_TN.getRPY(r_, p_, yaw_);
    double yawDesiredInDeg = yaw_ / DEG2RAD;
    
    // set Joystick mode
    FlightController::JoystickMode joystickMode = {
      FlightController::HorizontalLogic::HORIZONTAL_POSITION,
      FlightController::VerticalLogic::VERTICAL_POSITION,
      FlightController::YawLogic::YAW_ANGLE,
      FlightController::HorizontalCoordinate::HORIZONTAL_BODY,
      FlightController::StableMode::STABLE_ENABLE,
    };
    vehicle->flightController->setJoystickMode(joystickMode);

    // get current pose
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentGPSPosition =
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    Telemetry::TypeMap<TOPIC_QUATERNION>::type currentQuaternion =
        vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    Telemetry::GlobalPosition currentBroadcastGP = 
        vehicle->broadcast->getGlobalPosition();

    tf::Quaternion q_cur_TN(currentQuaternion.q1,
                        currentQuaternion.q2,
                        currentQuaternion.q3,
                        currentQuaternion.q0);
    tf::Matrix3x3 R_cur_TN(q_cur_TN);
    
    // Cal offset position from home to current      
    Vector3f localOffset_TN = localOffsetFromGpsAndFusedHeightOffset(
                                    currentGPSPosition, homeGPSPosition,
                                    currentBroadcastGP.height, homeHeight);
    // Cal offset position from current to desired
    Vector3f offsetRemaining_TN = vector3FSub(offsetDesired_TN, localOffset_TN);

    // transform yaw-oriented xy-coordinate
    tf::Vector3 pos_cmd_TN(offsetRemaining_TN.x,
                        offsetRemaining_TN.y,
                        offsetRemaining_TN.z);
    tf::Vector3 rotated_pos_cmd_TN(R_cur_TN.inverse() * pos_cmd_TN);
    Vector3f positionCommand;
    positionCommand.x = rotated_pos_cmd_TN.x();
    positionCommand.y = rotated_pos_cmd_TN.y();
    positionCommand.z = rotated_pos_cmd_TN.z();

    int xy_bound = 2;  // linear velocity bound
    horizCommandLimit(xy_bound, positionCommand.x, positionCommand.y);

    FlightController::JoystickCommand joystickCommand = {
        positionCommand.x, positionCommand.y,
        offsetDesired_TN.z + homeHeight, float(yawDesiredInDeg)};
    vehicle->flightController->setJoystickCommand(joystickCommand);
    vehicle->flightController->joystickAction();
  }



anbo_class::anbo_class(const ros::NodeHandle& n) : nh(n)    {
    ROS_WARN("Class generating...");
    getParam();
    
    task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");
    // auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
    // auto get_go_home_altitude_client = nh.serviceClient<GetGoHomeAltitude>("get_go_home_altitude");
    // auto set_current_point_as_home_client = nh.serviceClient<SetCurrentAircraftLocAsHomePoint>("/set_current_aircraft_point_as_home");
    // auto enable_horizon_avoid_client  = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
    // auto enable_upward_avoid_client   = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
    // auto get_avoid_enable_client      = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
    obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
    emergency_brake_client       = nh.serviceClient<dji_osdk_ros::EmergencyBrake>("emergency_brake");

    obtainCtrlAuthority.request.enable_obtain = true;
    obtain_ctrl_authority_client.call(obtainCtrlAuthority);

    quaternionSub = nh.subscribe("dji_osdk_ros/attitude", 10, &anbo_class::quaternionCallback, this);
    localPositionSub = nh.subscribe("dji_osdk_ros/local_position", 10, &anbo_class::localPositionCallback, this);
    missionStringSub = nh.subscribe("/mission_flag", 10, &anbo_class::missionDataCallback, this);
}

anbo_class::~anbo_class()   {
}


void anbo_class::getParam() {
    // position control
    nh.getParam("/waypoints", p_waypoints_in); // x, y, z, yaw
    nh.getParam("/pos_Threshold", p_th_pos);
    nh.getParam("/yaw_Threshold", p_th_yaw);

    // velocity control
    nh.getParam("/velocity_cmd", p_velo_in);   // x, y, z, yaw, time
}



//// Callback

void anbo_class::quaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& q_in){
    geometry_msgs::QuaternionStamped q = *q_in;
    c_quat_cur.q0 = q.quaternion.w;
    c_quat_cur.q1 = q.quaternion.x;
    c_quat_cur.q2 = q.quaternion.y;
    c_quat_cur.q3 = q.quaternion.z;
}

void anbo_class::localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_pos_in){
    c_local_pos_cur = *local_pos_in;
}

void anbo_class::missionDataCallback(const std_msgs::String::ConstPtr &string_msg){
    if (string_msg->data == "a"){
        std::cout << "| [a] Takeoff, Landing Test" << std::endl;
        test_takeoff_landing();
    }
    else if (string_msg->data == "b"){
        std::cout << "| [b] Position Control Test" << std::endl;
        test_position_control();
    }
    else if (string_msg->data == "c"){
        std::cout << "| [c] Velocity Control Test" << std::endl;
        test_velocity_control();
    }
}

//// Service Call

bool anbo_class::call_emergency_brake(float sleep_time) {
    ROS_INFO("EmergencyBrake for %fs", sleep_time);
    emergency_brake_client.call(emergency_brake);
    if(emergency_brake.response.result == true)    {
        ROS_INFO_STREAM("EmergencyBrake successful");
        ros::Duration(sleep_time).sleep();
    }
    else{
        ROS_ERROR_STREAM("EmergencyBrake failed");
    }
    return emergency_brake.response.result;
}

bool anbo_class::call_takeoff(float sleep_time) {
    control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
    ROS_INFO_STREAM("Takeoff request sending ...");
    if (task_control_client.call(control_task)) {
        ROS_INFO_STREAM("Takeoff task successful");
        ros::Duration(sleep_time).sleep();
    }
    else{
        ROS_ERROR_STREAM("Takeoff task failed");
    }
    return control_task.response.result;
}

bool anbo_class::call_landing() {
    control_task.request.task = FlightTaskControl::Request::TASK_LAND;
    ROS_INFO_STREAM("Landing request sending ...");
    if(task_control_client.call(control_task))  {
        ROS_INFO_STREAM("Landing task successful");
    }
    else{
        ROS_ERROR_STREAM("Landing task failed");
    }
    return control_task.response.result;
}

bool anbo_class::call_position_control()    {
    if(p_waypoints_in.size() < 4)   {
        ROS_ERROR_STREAM("Waypoints are incorrect");
        return false;
    }
    float yaw_origin = (quaternionToEulerAngle(c_quat_cur).z * 180.0 / M_PI) - 90.0;
    std::cout << "yaw_origin: " << yaw_origin << std::endl;
    float yaw_past = yaw_origin;
    float yaw_curr = yaw_past;
    for (int i = 0; i < p_waypoints_in.size()/4; i++) {
        
        ROS_INFO("start!");
        std::cout << "seq: " << i << std::endl;
        int j = i * 4;
        JoystickCommand wp;
        wp.x = p_waypoints_in[j];
        wp.y = p_waypoints_in[j+1];
        wp.z = p_waypoints_in[j+2];
        yaw_curr = yaw_past + p_waypoints_in[j+3];
        wp.yaw = yaw_curr;

        std::cout << "x: " << wp.x << std::endl;
        std::cout << "y: " << wp.y << std::endl;
        std::cout << "z: " << wp.z << std::endl;
        std::cout << "yaw: " << wp.yaw << std::endl;

        clock_t start, end;
        double duration;
        start = clock();
        move_pos_offset(control_task, wp, p_th_pos, p_th_yaw);
        end = clock();
        duration = (double)(end - start) / CLOCKS_PER_SEC;
        cout << duration << "sec" << endl;
        ROS_INFO_STREAM("Step over!");
        yaw_past = yaw_curr;
    }
    return control_task.response.result;
}

bool anbo_class::move_pos_offset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                    float pos_th, float yaw_th)  {
    task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    task.request.joystickCommand.x = offsetDesired.x;
    task.request.joystickCommand.y = offsetDesired.y;
    task.request.joystickCommand.z = offsetDesired.z;
    task.request.joystickCommand.yaw = offsetDesired.yaw;
    task.request.posThresholdInM   = pos_th;
    task.request.yawThresholdInDeg = yaw_th;

    task_control_client.call(task);
    return task.response.result;
}

bool anbo_class::call_velocity_control()    {
    if(p_velo_in.size() < 5)   {
        ROS_ERROR_STREAM("velocity commands are incorrect");
        return false;
    }

    for (int i = 0; i < p_velo_in.size()/5; i++) {
        int j = i * 5;
        JoystickCommand v_cmd;
        int take_time_ms;
        v_cmd.x = p_velo_in[j];
        v_cmd.y = p_velo_in[j+1];
        v_cmd.z = p_velo_in[j+2];
        v_cmd.yaw = p_velo_in[j+3];
        take_time_ms = p_velo_in[j+4];

        float yaw_origin = quaternionToEulerAngle(c_quat_cur).z * 180.0 / M_PI;

        control_task.request.task = FlightTaskControl::Request::TASK_VELOCITY_AND_YAWRATE_CONTROL;
        control_task.request.joystickCommand.x = v_cmd.x;   // max: 30 [m/s]
        control_task.request.joystickCommand.y = v_cmd.y;
        control_task.request.joystickCommand.z = v_cmd.z;
        control_task.request.joystickCommand.yaw = v_cmd.yaw;  // max: 150 [deg/s]
        control_task.request.velocityControlTimeMs   = take_time_ms;

        if(task_control_client.call(control_task))  {
            ROS_INFO("%d sequence success!!", i);
            ros::Duration(1.0).sleep();
            continue;
        }
        else{
            call_emergency_brake(2.0);
            ROS_ERROR_STREAM("Velocity Control failed");
            return control_task.response.result;
        }
    }
    return control_task.response.result;
}

//// Test

bool anbo_class::test_takeoff_landing() {
    ROS_INFO_STREAM("Takeoff and Landing Test...");
    if(call_takeoff(2.0))   {
        if(call_landing())  {
            ROS_INFO_STREAM("Return to Menu");
        }
    }
    return true;
}

bool anbo_class::test_position_control() {
    // ROS_INFO_STREAM("Position Control Test...");
    // if(call_takeoff(2.0))   {
    //     if(call_position_control())  {
    //         if(call_landing())  {
    //             ROS_INFO_STREAM("Return to Menu");
    //         }
    //     }
    // }
    // return true;
    control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
    ROS_INFO_STREAM("Takeoff request sending ...");
    task_control_client.call(control_task);
    if(control_task.response.result == false)
    {
        ROS_ERROR_STREAM("Takeoff task failed");
        return control_task.response.result;
    }

    if(control_task.response.result == true)
    {
        ROS_INFO_STREAM("Takeoff task successful");
        ros::Duration(2.0).sleep();

        ROS_INFO_STREAM("Move by position offset request sending ...");
        move_pos_offset(control_task, {0.0, 0.0, 6.0, 0.0}, 0.8, 1.0);
        ROS_INFO_STREAM("Step 1 over!");
        move_pos_offset(control_task, {6.0, 0.0, 0.0, 0.0}, 0.8, 1.0);
        ROS_INFO_STREAM("Step 2 over!");
        move_pos_offset(control_task, {-6.0, -6.0, 0.0, 0.0}, 0.8, 1.0);
        ROS_INFO_STREAM("Step 3 over!");
        move_pos_offset(control_task, {0.0, 0.0, 0.0, 90.0}, 0.8, 1);
        ROS_INFO_STREAM("Step 4 over!");
        move_pos_offset(control_task, {0.0, 6.0, 0.0, 90.0}, 0.8, 1);
        ROS_INFO_STREAM("Step 5 over!");
        move_pos_offset(control_task, {-6.0, 0.0, 0.0, 90.0}, 0.8, 1);
        ROS_INFO_STREAM("Step 6 over!");

        control_task.request.task = FlightTaskControl::Request::TASK_LAND;
        ROS_INFO_STREAM("Landing request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
            ROS_INFO_STREAM("Land task successful");
            return control_task.response.result;
        }
        ROS_INFO_STREAM("Land task failed.");
        return control_task.response.result;
    }
    return control_task.response.result;
}

bool anbo_class::test_velocity_control() {
    ROS_INFO_STREAM("Velocity Control Test...");
    if(call_takeoff(2.0))   {
        if(call_velocity_control())  {
            if(call_landing())  {
                ROS_INFO_STREAM("Return to Menu");
            }
        }
    }
    return true;
}

//// Main

int main(int argc, char** argv) {
    ros::init(argc, argv, "anbo_node");
    ros::NodeHandle n("~");
    anbo_class anbo_object_(n);
    // anbo_object_.menu();

    ros::AsyncSpinner spinner(4); // Use 8 threads -> 3 callbacks + 2 Timer callbacks + 1 spare threads for publishers
    spinner.start();
    ros::waitForShutdown();
    return 0;
}