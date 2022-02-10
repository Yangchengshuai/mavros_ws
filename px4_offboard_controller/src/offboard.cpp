#include "offboard.hpp"

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void scan_tf_callback(const sensor_msgs::LaserScan& msg){
    if(scan_chixu_cnt<2)
    {
        scan_chixu_cnt++;
    }
    if (scan_chixu_cnt>1)
    {
        scan_msg = msg;
        scan_msg_flag = true;
    }
    scan_noiscnt++;
    if(scan_noiscnt>1){
        scan_msg_last = msg;
        scan_noiscnt = 0;
    }
}

void odometry_callback(const nav_msgs::Odometry &current_info)
{
    current_point.x= current_info.pose.pose.position.x;
    current_point.y= current_info.pose.pose.position.y;
    current_point.z= current_info.pose.pose.position.z;
    current_angle.x= current_info.pose.pose.orientation.x;
    current_angle.y= current_info.pose.pose.orientation.y;
    current_angle.z= current_info.pose.pose.orientation.z;
    current_angle.w= current_info.pose.pose.orientation.w;
    curr_cam_angle = unionsys_core->toEulerAngle(current_angle);
    
    // pitch down 45 and yaw 180 degree , quaternion of plane to camera
    camera_setup_angle.x = 0;
    camera_setup_angle.y = 0.7854;
    camera_setup_angle.z = 3.1416;

    geometry_msgs::Quaternion quaternion_cp=tf::createQuaternionMsgFromRollPitchYaw(camera_setup_angle.x,camera_setup_angle.y,camera_setup_angle.z);

    geometry_msgs::Quaternion q_p_g = unionsys_core->quatMultiply(quaternion_cp, current_angle);
    curr_plane_angle = unionsys_core->toEulerAngle(q_p_g);

    //rotation_hehe_data =  -Rcg*Rcp*Pcp
    geometry_msgs::Point rotation_hehe_data = unionsys_core->rotate_Rcg_Ppc(curr_cam_angle, camera_setup_angle, DELTA_X, DELTA_Y, DELTA_Z);
    current_point.x += rotation_hehe_data.x;
    current_point.y += rotation_hehe_data.y;
    current_point.z += rotation_hehe_data.z;
    //publish plane odom
    nav_msgs::Odometry plane_odom;
    plane_odom = current_info;
    plane_odom.pose.pose.position.x = current_point.x;
    plane_odom.pose.pose.position.y = current_point.y;
    plane_odom.pose.pose.position.z = current_point.z;
    // geometry_msgs::Quaternion quaternion_tmp=tf::createQuaternionMsgFromRollPitchYaw(curr_plane_angle.x,curr_plane_angle.y,curr_plane_angle.z);
    plane_odom.pose.pose.orientation.x=q_p_g.x;
    plane_odom.pose.pose.orientation.y=q_p_g.y;
    plane_odom.pose.pose.orientation.z=q_p_g.z;
    plane_odom.pose.pose.orientation.w=q_p_g.w;
    pose_transform_pub.publish(plane_odom);

    //publish camera/pose
    geometry_msgs::PoseStamped pose_tf;
    //need to be change the rotate angle ,becuse of the setup formation of d435i 
    //pitch -90
    Eigen::Matrix3d Rpitch;
        Rpitch<< cos(-1.57), 0,  -sin(-1.57),
                    0,       1,     0,
                sin(-1.57),  0, cos(-1.57);
    // roll -90
    Eigen::Matrix3d Rroll;
    Rroll<< 1, 0, 0,
            0, cos(-1.57), sin(-1.57),
            0, -sin(-1.57), cos(-1.57);


    // pose_tf = unionsys_core->calculate_cam_pos(plane_odom,1.57,0,1.57);
    pose_tf = unionsys_core->calculate_cam_pos(plane_odom,Rroll,Rpitch);

    pose_tf.header.stamp = ros::Time::now();
    pose_tf.header.frame_id = "world";

    pose_tf_pub.publish(pose_tf);
}

void local_odom_callback(const nav_msgs::Odometry & target_att)
{
    current_z_speed = target_att.twist.twist.linear.z;
}

void rcin_callback(const mavros_msgs::RCIn & rcvalue)
{
    channel5_value = rcvalue.channels[4];
    channel6_value = rcvalue.channels[5];
    channel7_value = rcvalue.channels[6];
    channel8_value = rcvalue.channels[7];
}

void planWaypoint_callback(const quadrotor_msgs::PositionCommand &msg){
    planned_point.x= msg.position.x;
    planned_point.y= msg.position.y;
    planned_point.z= msg.position.z;
    planned_velocity.x = msg.velocity.x;
    planned_velocity.y = msg.velocity.y;
    planned_velocity.z = msg.velocity.z;
    planned_yaw_angle = msg.yaw;
    planned_yaw_dot = msg.yaw_dot;
    planned_update_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_board_test");
    ros::NodeHandle nh;
    unionsys_core->detect_serial_number();

    //read parameters from launch file
    nh.param<bool>("IFPLANNER",IFPLANNER,false);
    nh.param<double>("lidar_avoid_distance",lidar_avoid_distance,0.7);

    odometrysub     = nh.subscribe("/camera/odom/sample",10,odometry_callback);
    planWaypointsub = nh.subscribe("/planning/pos_cmd", 10, planWaypoint_callback);
    local_odom_sub = nh.subscribe("/mavros/local_position/odom",10,local_odom_callback);
    rcin_sub = nh.subscribe("/mavros/rc/in",10,rcin_callback);
    state_sub = nh.subscribe("mavros/state", 10, state_cb);
    scan_tf_sub = nh.subscribe("/scan", 10, scan_tf_callback);

    pose_tf_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera/pose", 10);
    pose_transform_pub = nh.advertise<nav_msgs::Odometry>("/plane_odom", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    aux_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_vechile_ned = nh.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        std::cout<<"------------>wait for fcu connected..."<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMavFrame setmavframe;
    setmavframe.request.mav_frame = 8;
 
    ros::Time last_request = ros::Time::now();
    if(set_vechile_ned.call(setmavframe) && setmavframe.response.success)
    {
        std::cout<<"------------>setmavframe success..."<<std::endl;
    }
    else{
        std::cout<<"------------>setmavframe failed"<<std::endl;
    }
    
    while(ros::ok()){
        //if use sitl ,do this

        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } 
        // else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
                
        //         last_request = ros::Time::now();
        //     }
        // }

        // mavros_msgs::ActuatorControl out_aux;
        // out_aux.header.seq = 1;
        // out_aux.header.stamp = ros::Time::now();
        // out_aux.header.frame_id = "aux";
        // out_aux.group_mix = 3;
        // out_aux.controls = {1900.0,1900.0,1900.0,1900.0,1900.0,1900.0,1900.0,1900.0};
        // aux_pub.publish(out_aux);

        
        if (IFPLANNER){
            if (planned_update_flag){
                position_pid_control(planned_point,current_point,1.5,planned_yaw_angle,0,7);
                linear_tmp.x = velocity_expected.x;
                linear_tmp.y = velocity_expected.y;
                linear_tmp.z = velocity_expected.z;
                angular_tmp.x = 0;
                angular_tmp.y = 0;
                angular_tmp.z = attitude_expect.z;

                msgtwist.header.stamp = ros::Time::now();
                msgtwist.header.seq=1;
                msgtwist.twist.linear=linear_tmp;
                msgtwist.twist.angular=angular_tmp;
                // ROS_INFO("send to vechile: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                local_vel_pub.publish(msgtwist);
                planned_update_flag = false;
            
            }
            else{
                position_pid_control(current_point,current_point,1.5,0,0,7);
                linear_tmp.x = velocity_expected.x;
                linear_tmp.y = velocity_expected.y;
                linear_tmp.z = velocity_expected.z;
                angular_tmp.x = 0;
                angular_tmp.y = 0;
                angular_tmp.z = attitude_expect.z;
                msgtwist.header.stamp = ros::Time::now();
                msgtwist.header.seq=1;
                msgtwist.twist.linear=linear_tmp;
                msgtwist.twist.angular=angular_tmp;
                local_vel_pub.publish(msgtwist);
                // ROS_INFO("no planned message: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                // ROS_INFO("no planned message");
            }
        }
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}


void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode)
{
    float vx = 0.0, vy = 0.0, vxp = 0.0, vyp = 0.0, vxi = 0.0, vyi = 0.0, vxd = 0.0, vyd = 0.0, vx_lim=0.0, vy_lim=0.0;
    float yaw_rate = 0.0, yaw_rate_p = 0.0, yaw_rate_i = 0.0, yaw_rate_d = 0.0, vz = 0;
    float roll = 0.0, pitch = 0.0;
    //position control  mode & 0x01
    if (1) {
        //calculate velocity, P control
        geometry_msgs::Point error_pos;
        error_pos.x = current_set_point.x - current_local_point.x;
        error_pos.y = current_set_point.y - current_local_point.y;
        vxp = P_pos * error_pos.x;
        vyp = P_pos * error_pos.y;
        vxd = D_pos * (error_pos.x - error_pos_last.x);
        vyd = D_pos * (error_pos.y - error_pos_last.y);
        vxd = limit(vxd, D_VEL_LIMIT);
        vyd = limit(vyd, D_VEL_LIMIT);
        /*if(abs(error_pos.x) < abs(error_pos_last.x) && (abs(error_pos.x) >= 1.5 * DEAD_ZONE))
        {
          vxd = 0.0;
        }
        if(abs(error_pos.y) < abs(error_pos_last.y) && (abs(error_pos.y) >= 1.5 * DEAD_ZONE))
        {
          vyd = 0.0;
        }*/
        if (abs(error_pos.x) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.x += error_pos.x;
        } else {
            error_pos_integrated.x = 0.0;
        }
        if (abs(error_pos.y) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.y += error_pos.y;
        } else {
            error_pos_integrated.y = 0.0;
        }
        if (I_pos > 0.0001) {
            error_pos_integrated.x = limit((float) error_pos_integrated.x, I_VEL_LIMIT / I_pos);
            error_pos_integrated.y = limit((float) error_pos_integrated.y, I_VEL_LIMIT / I_pos);
        }
        vxi = I_pos * error_pos_integrated.x;
        vyi = I_pos * error_pos_integrated.y;
        vx = (vxp + vxi + vxd);
        vy = (vyp + vyi + vyd);

        float x_offset = current_set_point.x - current_local_point.x;
        float y_offset = current_set_point.y - current_local_point.y;
        float distance = sqrt(x_offset * x_offset + y_offset * y_offset);
        if (distance <= dead_zone) {
            dead_zone_flag = true;
            error_pos_integrated.x = 0.0;
            error_pos_integrated.y = 0.0;
            vx = 0;
            vy = 0;
        } else {
            dead_zone_flag = false;
        }

        //limit the speed
        vx_lim = unionsys_core->limit_velocity(vx, vy, velocity_limit).x;
        vy_lim = unionsys_core->limit_velocity(vx, vy, velocity_limit).y;
        //ROS_INFO("vx_exp vy_exp %f %f",vx,vy);

        error_pos_last = error_pos;
    }
    //yaw control  mode & 0x02
    if (1) {
        //use the flight controller yaw in default
        
        float current_yaw = curr_plane_angle.z;
        //Drone turns at the smallest angle
        float error_yaw = (target_yaw - current_yaw) * rad2deg;
        if (error_yaw < -180)
            error_yaw += 360;
        else if (error_yaw > 180)
            error_yaw -= 360;
        yaw_rate_p = P_yaw * error_yaw;
        yaw_rate_d = D_yaw * (error_yaw - error_yaw_last);
        error_yaw_integrated += error_yaw;
        if (I_yaw > 0.0001) {
            error_yaw_integrated = limit(error_yaw_integrated, YAW_I_LIMIT / I_yaw);
        }
        yaw_rate_i = I_yaw * error_yaw_integrated;
        if (abs(error_yaw) <= YAW_DEAD_ZONE) {
            yaw_rate_p = 0.0;
            yaw_rate_i = 0.0;
            yaw_rate_d = 0.0;
            error_yaw_integrated = 0.0;
        }
        yaw_rate = (yaw_rate_p + yaw_rate_i + yaw_rate_d);
        if (abs(error_yaw) >= YAW_RATE_LIMIT) {
            yaw_rate = limit(yaw_rate, YAW_RATE_LIMIT);
        } 
        // else {
        //     yaw_rate = limit(yaw_rate, (float) (YAW_RATE_LIMIT * 0.4));
        // }
        error_yaw_last = error_yaw;
    }

    //height control  mode & 0x04
    if (1) {
        //control the height around 1.5m, the height is from flight controller
        float deltaz = current_local_point.z - current_set_point.z;
        if (fabs(deltaz) >= HEIGHT_DEAD_ZONE) {
            vz = P_z * (current_set_point.z - current_local_point.z);
        } else {
            vz = 0.0;
        }
    }

    //hah

    bizhangvx = 0;
    bizhangvy = 0;

    //a1 start this
    // if(scan_msg_flag){
    //     for(int i=0;i<360;i++){
    //         //距离设置 0.7m
    //         // ROS_INFO("----->scan msg: %f", scan_msg.ranges[i]);
    //         if(scan_msg.ranges[i]<lidar_avoid_distance && scan_msg.ranges[i]>0.1 && scan_msg_last.ranges[i]<lidar_avoid_distance && scan_msg_last.ranges[i]>0.1){
    //             ROS_INFO("--------i is : %d", i);
    //             bizhangvy += sin(i*deg2rad)/scan_msg.ranges[i];
    //             bizhangvx += cos(i*deg2rad)/scan_msg.ranges[i];    
    //         }
    //     }
        
    //     scan_msg_flag = false;
    // }

    //s1 start this   ,because s1 has 720 points instead of 360
    if(scan_msg_flag){
        for(int i=0;i<720;i++){
            //距离设置 0.7m
            // ROS_INFO("----->scan msg: %f", scan_msg.ranges[i]);
            if(scan_msg.ranges[i]<lidar_avoid_distance && scan_msg.ranges[i]>0.1 && scan_msg_last.ranges[i]<lidar_avoid_distance && scan_msg_last.ranges[i]>0.1){
                ROS_INFO("--------i is : %d", i);
                bizhangvy += sin(i*deg2rad/2)/scan_msg.ranges[i];
                bizhangvx += cos(i*deg2rad/2)/scan_msg.ranges[i];    
            }
        }
        
        scan_msg_flag = false;
    }
    
    // ROS_INFO("--------------------p");
    if( fabs(bizhangvx)>0 || fabs(bizhangvy)>0){
        bizhangvx = -bizhangvx*0.01;
        bizhangvy = -bizhangvy*0.01;
        ROS_INFO("bizhang speed no limit : %f %f", bizhangvx, bizhangvy);
        velocity_expected.x = limit(bizhangvx,BIZHANG_LIMIT);
        velocity_expected.y = limit(bizhangvy,BIZHANG_LIMIT);
        ROS_INFO("bizhang speed limited: %f %f", velocity_expected.x, velocity_expected.y);
        velocity_expected.z = 0;
        attitude_expect.z = 0;
    }
    else{
        velocity_expected.x = vx_lim * cos(curr_plane_angle.z) + vy_lim * sin(curr_plane_angle.z);
        velocity_expected.y = vy_lim * cos(curr_plane_angle.z) - vx_lim * sin(curr_plane_angle.z);
        velocity_expected.z = vz;
        attitude_expect.z = yaw_rate * deg2rad;
    }

}
