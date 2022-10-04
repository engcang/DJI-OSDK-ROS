#include <dji_osdk_ros/anbo_node.h>
#include <time.h>

using namespace dji_osdk_ros;

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