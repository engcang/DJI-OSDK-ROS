

/*
  EXAMPLE CODE from Eungchang Mason Lee (eungchang_mason@kaist.ac.kr)
* Code: GPS health → local frame set → manual arming → authority + take off → position control
* Mode change in remote controller: ctrl authrotiy take over
* Left back Gimbal channel in remote controller: ctrl authority hand over back to code
*/


///// common headers
#include <math.h>
#include <cmath>

///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)

///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/GimbalAction.h>

///// Ctrl+C
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;
using namespace dji_osdk_ros;



class dji_controller_class{
  public:
    ///// Drone, control
    Eigen::Matrix<double, 4, 4> m_map_t_body = Eigen::Matrix<double, 4, 4>::Identity();
    double m_curr_yaw;
    bool m_gps_status_ok=false;
    bool m_pose_check=false;
    bool m_ctrl_init=false;
    uint8_t m_flight_status=0;
    int square_idx=0;
    ros::Time m_ctrl_time_t;

    ///// cam
    double m_cam_capture_tmp_time_counter=0.0;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber m_pose_sub, m_gps_status_sub, m_flight_status_sub, m_rc_sub;
    ros::Publisher m_position_controller_pub, m_cam_capture_pub;
    ros::ServiceClient m_local_ref_set_client, m_control_authority_client, m_flight_task_client, m_gimbal_client;
    ros::Timer m_control_timer, m_gimbal_control_timer;

    ///// functions
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void gps_status_cb(const std_msgs::UInt8::ConstPtr& msg);
    void flight_status_cb(const std_msgs::UInt8::ConstPtr& msg);
    void rc_sub(const sensor_msgs::Joy::ConstPtr& msg);
    void control_timer_func(const ros::TimerEvent& event);
    void gimbal_control_timer_func(const ros::TimerEvent& event);

    dji_controller_class(const ros::NodeHandle& n_private) : nh(n_private){
      ///// ROS      
      // publishers
      m_position_controller_pub = nh.advertise<geometry_msgs::PoseStamped>("/dji_osdk_ros/set_local_pose", 3);
      m_cam_capture_pub = nh.advertise<std_msgs::Empty>("/shooting", 3);
      // subscribers
      m_pose_sub = nh.subscribe<nav_msgs::Odometry>("/dji_osdk_ros/local_odom", 3, &dji_controller_class::pose_cb, this);
      m_gps_status_sub = nh.subscribe<std_msgs::UInt8>("/dji_osdk_ros/gps_health", 3, &dji_controller_class::gps_status_cb, this);
      m_flight_status_sub = nh.subscribe<std_msgs::UInt8>("/dji_osdk_ros/flight_status", 3, &dji_controller_class::flight_status_cb, this);
      m_rc_sub = nh.subscribe<sensor_msgs::Joy>("/dji_osdk_ros/rc", 3, &dji_controller_class::rc_sub, this);
      // services
      m_local_ref_set_client = nh.serviceClient<dji_osdk_ros::SetLocalPosRef>("/set_local_pos_reference");
      m_control_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("/obtain_release_control_authority");
      m_flight_task_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("/flight_task_control");
      m_gimbal_client = nh.serviceClient<dji_osdk_ros::GimbalAction>("/gimbal_task_control");
      // timer
      m_control_timer = nh.createTimer(ros::Duration(1/20.0), &dji_controller_class::control_timer_func, this);
      m_gimbal_control_timer = nh.createTimer(ros::Duration(1/1.0), &dji_controller_class::gimbal_control_timer_func, this);
      
      ROS_WARN("Starting Code...");
    }
};









//////////////// can be seperated into .cpp files
//////////////////////// callbacks
void dji_controller_class::pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose pose = msg->pose.pose;
  Eigen::Matrix<double, 4, 4> tmp_mat = Eigen::Matrix<double, 4, 4>::Identity();

  tmp_mat.block<3, 3>(0, 0) = Eigen::Quaternion<double>(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
  tmp_mat.block<3, 1>(0, 3) = Eigen::Matrix<double, 3, 1>(pose.position.x, pose.position.y, pose.position.z);

  m_map_t_body = tmp_mat;
  tf::Matrix3x3 mat(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
  double _r, _p;
  mat.getRPY(_r, _p, m_curr_yaw);
  m_pose_check=true;
}
void dji_controller_class::gps_status_cb(const std_msgs::UInt8::ConstPtr& msg)
{
  if (!m_gps_status_ok)
  {
    if ( msg->data > 3u )
    {
      ROS_WARN("GPS health OK: %lu", msg->data);
      dji_osdk_ros::SetLocalPosRef srv_;
      m_local_ref_set_client.call(srv_);
      if (srv_.response.result)
      {
        ROS_WARN("Local ref set OK");
        m_gps_status_ok=true;
      }
    }
    else
    {
      ROS_WARN("GPS health is not good!");
    }
  }
}
void dji_controller_class::flight_status_cb(const std_msgs::UInt8::ConstPtr& msg)
{
  m_flight_status = msg->data;
  if (m_gps_status_ok && m_pose_check)
  {
    if (m_flight_status == 0u && !m_ctrl_init)
    {
      ROS_INFO("Waiting for arming...");
      ros::Duration(0.2).sleep();
    }
    else if (m_flight_status == 1u && !m_ctrl_init)
    {
      ROS_WARN("Armed, trying to obtain ctrl authority");
      dji_osdk_ros::ObtainControlAuthority srv_;
      srv_.request.enable_obtain=true;
      m_control_authority_client.call(srv_);
      if (srv_.response.result)
      {
        ROS_WARN("Obtained authority, disarming then taking off");
        dji_osdk_ros::FlightTaskControl srv2_;
        srv2_.request.task=8;
        m_flight_task_client.call(srv2_);
        if (srv2_.response.result)
        {
          ros::Duration(1.0).sleep();
          dji_osdk_ros::FlightTaskControl srv3_;
          srv3_.request.task=4;
          m_flight_task_client.call(srv3_);
          if (srv3_.response.result)
          {
            ROS_WARN("Took off, offboard start");
            m_ctrl_time_t = ros::Time::now();
            m_ctrl_init=true;
          }
        }
      }
    }
    else if (m_flight_status == 2u && m_ctrl_init)
    {
      if ( (ros::Time::now()-m_ctrl_time_t) > ros::Duration(60.0) )
      {
        ROS_WARN("Time out, homing and landing!");
        m_ctrl_time_t = ros::Time::now();
        dji_osdk_ros::FlightTaskControl srv_;
        srv_.request.task=6;
        m_flight_task_client.call(srv_);
      }
    }
  }
}
void dji_controller_class::rc_sub(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->axes.size()>0)
  {
    if (fabs(msg->axes[5]) > 5000.0)
    {
      dji_osdk_ros::ObtainControlAuthority srv_;
      srv_.request.enable_obtain=true;
      m_control_authority_client.call(srv_);
    }
  }
}


//////////////////////// timers
void dji_controller_class::gimbal_control_timer_func(const ros::TimerEvent& event)
{
  if (m_flight_status == 2u && m_ctrl_init)
  {
    dji_osdk_ros::GimbalAction srv_;
    srv_.request.header.stamp = ros::Time::now();
    srv_.request.is_reset = false;
    srv_.request.payload_index = 0u;
    srv_.request.rotationMode = 1u;
    srv_.request.pitch = 0.0f;
    srv_.request.roll = 0.0f;
    srv_.request.yaw = 0.0f;
    srv_.request.time = 1.0f;
    m_gimbal_client.call(srv_);
  }
}
void dji_controller_class::control_timer_func(const ros::TimerEvent& event)
{
  if (m_gps_status_ok && m_pose_check && m_ctrl_init && m_flight_status==2u)
  {
    tf::Matrix3x3 mat;
    tf::Quaternion quat;
    geometry_msgs::PoseStamped pose_in;
    pose_in.header.stamp = ros::Time::now();
    double des_yaw=0.0;
    if (square_idx%4==0)
    {
      des_yaw = 0.0;
      pose_in.pose.position.x = 2.0;
      pose_in.pose.position.y = 0.0;
    }
    else if (square_idx%4==1)
    {
      des_yaw = 1.5708;
      pose_in.pose.position.x = 2.0;
      pose_in.pose.position.y = 2.0;
    }
    else if (square_idx%4==2)
    {
      des_yaw = 3.1415;
      pose_in.pose.position.x = 0.0;
      pose_in.pose.position.y = 2.0;
    }
    else if (square_idx%4==3)
    {
      des_yaw = -1.5708;
      pose_in.pose.position.x = 0.0;
      pose_in.pose.position.y = 0.0;
    }
    mat.setRPY(0.0, 0.0, des_yaw);
    mat.getRotation(quat);
    pose_in.pose.position.z = 1.5;
    pose_in.pose.orientation.x = quat.getX();
    pose_in.pose.orientation.y = quat.getY();
    pose_in.pose.orientation.z = quat.getZ();
    pose_in.pose.orientation.w = quat.getW();
    m_position_controller_pub.publish(pose_in);

    if (sqrt(pow(pose_in.pose.position.x-m_map_t_body(0,3), 2) + pow(pose_in.pose.position.y-m_map_t_body(1,3), 2) + pow(pose_in.pose.position.z-m_map_t_body(2,3), 2)) < 0.3 
      && fabs(m_curr_yaw - des_yaw) < 0.2)
    {
      std_msgs::Empty empty_;
      if (m_cam_capture_tmp_time_counter == 0.0)
        m_cam_capture_tmp_time_counter = ros::Time::now().toSec();
      else if (ros::Time::now().toSec() - m_cam_capture_tmp_time_counter > 0.6)
      {
        square_idx++;
        m_cam_capture_tmp_time_counter = 0.0;
      }
      else if (ros::Time::now().toSec() - m_cam_capture_tmp_time_counter > 0.3)
      {
        m_cam_capture_pub.publish(empty_);
      }
    }
  }
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_controller_node");
  ros::NodeHandle nh_private("~");

  dji_controller_class dji_controller_(nh_private);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::AsyncSpinner spinner(5); // Use 6 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}