#ifndef __ANBO_NODE_HH__
#define __ANBO_NODE_HH__

#include <ros/ros.h>
#include <dji_vehicle.hpp>

// #include <dji_osdk_ros/vehicle_wrapper.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/vehicle_wrapper.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#define state_Takeoff_Landing 'a'
#define state_Position_Control 'b'
#define state_Velocity_Control 'c'
#define state_Quit 'q'

class anbo_class    {
    public:
        anbo_class();
        ~anbo_class();
        bool menu();

    private:
        //// ROS
        ros::NodeHandle nh;
        ros::Subscriber quaternionSub;
        ros::Subscriber localPositionSub;
        ros::Subscriber rcDataSub;
        ros::Subscriber missionStringSub;

        //// Service Client
        ros::ServiceClient task_control_client;
        ros::ServiceClient obtain_ctrl_authority_client;
        ros::ServiceClient emergency_brake_client;
        ros::ServiceClient set_joystick_mode_client;
        ros::ServiceClient joystick_action_client;

        //// Service object
        dji_osdk_ros::EmergencyBrake emergency_brake;
        dji_osdk_ros::FlightTaskControl control_task;
        dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
        dji_osdk_ros::SetJoystickMode joystickMode;
        dji_osdk_ros::JoystickAction joystickAction;

        ///// Jonna important Control Authority haha
        bool control_authority_check=false;

        //// Parameters
        // position control
        std::vector<double> p_waypoints_in;
        double p_th_pos;
        double p_th_yaw;
        // velocity control
        std::vector<double> p_velo_in;

        //// Callback Var
        Telemetry::Quaternion c_quat_cur;
        geometry_msgs::PointStamped c_local_pos_cur;
        sensor_msgs::Joy c_rc_data;

        //// Callback
        void quaternionCallback(const geometry_msgs::QuaternionStampedConstPtr& q_in);
        void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_pos_in);
        void rcDataCallback(const sensor_msgs::Joy::ConstPtr& rc_data_in);
        void missionDataCallback(const std_msgs::String::ConstPtr &string_msg);

        
        void setup();
        void getParam();

        bool call_emergency_brake(float sleep_time);
        bool call_takeoff(float sleep_time);
        bool call_landing();
        bool call_position_control();
        bool call_velocity_control();

        bool test_takeoff_landing();
        bool test_position_control();
        bool test_velocity_control();

        //// Utility
        Telemetry::Vector3f quaternionToEulerAngle(const Telemetry::Quaternion& quat)
        {
            Telemetry::Vector3f eulerAngle;
            double q2sqr = quat.q2 * quat.q2;
            double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
            double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
            double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
            double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
            double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
            t2 = (t2 > 1.0) ? 1.0 : t2;
            t2 = (t2 < -1.0) ? -1.0 : t2;
            eulerAngle.x = asin(t2);
            eulerAngle.y = atan2(t3, t4);
            eulerAngle.z = atan2(t1, t0);
            return eulerAngle;
        }
};

#endif // __ANBO_NODE_HH__