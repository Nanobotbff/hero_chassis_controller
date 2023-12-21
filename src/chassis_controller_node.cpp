#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control_toolbox/pid.h"
#include "iostream"

//#include "simple_chassis_controller/simple_chassis_controller.h"
//#include <pluginlib/class_list_macros.hpp>
//
//namespace simple_chassis_controller {
//    bool SimpleChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
//                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
//        front_left_joint_ =
//                effort_joint_interface->getHandle("left_front_wheel_joint");
//        front_right_joint_ =
//                effort_joint_interface->getHandle("right_front_wheel_joint");
//        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
//        back_right_joint_ =
//                effort_joint_interface->getHandle("right_back_wheel_joint");
//
//        return true;
//    }

control_toolbox::Pid pid_right_back_wheel;
control_toolbox::Pid pid_left_back_wheel;
control_toolbox::Pid pid_left_front_wheel;
control_toolbox::Pid pid_right_front_wheel;

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
    double right_back_wheel_vel = pid_right_back_wheel.computeCommand(msg->linear.x);
    double left_back_wheel_vel = pid_left_back_wheel.computeCommand(msg->linear.x);
    double left_front_wheel_vel = pid_left_front_wheel.computeCommand(msg->linear.x);
    double right_front_wheel_vel = pid_right_front_wheel.computeCommand(msg->linear.x);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_controller_node");
    ros::NodeHandle nh;

    double p, i, d;
    nh.getParam("/chassis_controller/right_back_wheel/p", p);
    nh.getParam("/chassis_controller/right_back_wheel/i", i);
    nh.getParam("/chassis_controller/right_back_wheel/d", d);
    pid_right_back_wheel.initPid(p, i, d);


    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdVelCallback);

    ros::spin();

    return 0;
}