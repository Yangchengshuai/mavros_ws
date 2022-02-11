#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>

#include <geometry_msgs/TwistStamped.h>
 
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h> 
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <sensor_msgs/LaserScan.h>  qaq
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ActuatorControl.h>
#include "quadrotor_msgs/PositionCommand.h"
// #include "off_board_test/gimbal.h"
#include <unionsys_core.hpp>

RegisterMSN *unionsys_core = new RegisterMSN();

//控制pid
const float P_pos = 1.0;
const float I_pos = 0.0;
const float D_pos = 0.0;
const float P_yaw = 1.00;
const float I_yaw = 0.00;
const float D_yaw = 0.00;
const float P_z = 1.00;

const float I_VEL_LIMIT = 0.025; //the intergrate item limit of position control, limit the expected velocity
const float D_VEL_LIMIT = 0.015;

const float YAW_I_LIMIT = 2.0;
const float YAW_RATE_LIMIT = 45.0;

const float BIZHANG_LIMIT = 0.5;

bool dead_zone_flag = false;

float current_z_speed = 0.0;

int channel5_value;
int channel6_value;
int channel7_value;
int channel8_value;

const float deg2rad = 3.1415926535798/180.0;
const float rad2deg = 180.0/3.1415926535798;

float bizhangvx = 0;
float bizhangvy = 0;
int scan_noiscnt = 0;
int scan_chixu_cnt = 0;

//launch file variables
bool IFPLANNER;
double lidar_avoid_distance;

//Position  camera on plane coord
#define DELTA_X -0.03
#define DELTA_Y -0.0
#define DELTA_Z -0.1

#define DEAD_ZONE 0.01 //dead zone, unit: m
#define POS_I_DEAD_ZONE 0.04 //in dead zone, don't intergrate
#define YAW_DEAD_ZONE 2
#define HEIGHT_DEAD_ZONE 0.0001

//ros node define
mavros_msgs::State current_state;
ros::Subscriber local_odom_sub;
ros::Subscriber rcin_sub;
ros::Subscriber state_sub;
ros::Subscriber odometrysub;
ros::Subscriber planWaypointsub;
ros::Subscriber scan_tf_sub;
ros::Publisher  local_vel_pub;
ros::Publisher  pose_tf_pub, pose_transform_pub, aux_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient set_vechile_ned;

geometry_msgs::Point current_point;
geometry_msgs::Point planned_point;
geometry_msgs::Point planned_velocity;
geometry_msgs::Vector3 current_imu_angle;
geometry_msgs::Quaternion current_angle;
geometry_msgs::Vector3 curr_cam_angle;
geometry_msgs::Vector3 curr_plane_angle;
geometry_msgs::Vector3 camera_setup_angle;



float error_yaw_last = 0.0, error_yaw_integrated = 0.0;
bool current_update_flag = false, planned_update_flag = false, scan_msg_flag = false;

sensor_msgs::LaserScan scan_msg;
sensor_msgs::LaserScan scan_msg_last;

float current_yaw_angle;
float planned_yaw_angle;
float planned_yaw_dot;

geometry_msgs::Point error_pos_last, error_pos_integrated, attitude_expect, velocity_expected;
geometry_msgs::TwistStamped msgtwist;
geometry_msgs::Vector3 linear_tmp;
geometry_msgs::Vector3 angular_tmp;


template <typename T>
T limit(T a, T max)
{
  if(a > max)
    a = max;
  if(a < -max)
    a = -max;
  return a;
}

void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode);
