#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control_toolbox/pid.h"

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

//example:
//#include "PID.h"
//PID::PID()
//{
//
//}
//
//void PID::update(int min_val, int max_val, float kp, float ki, float kd)
//{
//    min_val_ = min_val;
//    max_val_ = max_val;
//    kp_ = kp;
//    ki_ = ki;
//    kd_ = kd;
//}
//
//int PID::compute(int setpoint, int measured_value)
//{
//
//    double error = 0;
//    double pid = 0;
//
//    //setpoint is constrained between min and max to prevent pid from having too much error
//    if(setpoint == 0){
//        integral_ = 0;
//        derivative_ = 0;
//        prev_error_ = 0;
//        return 0;
//    }
//    error = setpoint - measured_value;
//    if(abs(error)<0.1)error=0;
//    integral_ += error;
//    derivative_ = error - prev_error_;
//    if(setpoint == 0 && error == 0){
//        integral_ = 0;
//    }
//    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
//    prev_error_ = error;
//
//    return constrain(pid, min_val_, max_val_);
//}
//void PID::updateConstants(float kp, float ki, float kd)
//{
//    kp_ = kp;
//    ki_ = ki;
//    kd_ = kd;
//}
//PID motor1_pid;
//PID motor2_pid;
//PID motor3_pid;
//PID motor4_pid;
//void setPidParam(void)
//{
//    switch(configParam.RobotType){
//        case 1:{//d2 t2
//            motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
//            motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
//        }break;
//        case 3:{//a1 转向舵机+两个减速电机
//            motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
//            motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
//        }break;
//        case 4:{//a2 转向舵机+一个动力电机
//            if(MotorType_t == M_ESC_ENC){
//                motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                                  configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
//            }
//        }break;
//        case 5:{//o3
//            motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
//            motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
//            motor3_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M3.K_p, configParam.p_M3.K_i,configParam.p_M3.K_d);
//        }break;
//        case 2: //d4 t4
//        case 6: //o4
//        case 7:{//m4
//            motor1_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M1.K_p, configParam.p_M1.K_i,configParam.p_M1.K_d);
//            motor2_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M2.K_p, configParam.p_M2.K_i,configParam.p_M2.K_d);
//            motor3_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M3.K_p, configParam.p_M3.K_i,configParam.p_M3.K_d);
//            motor4_pid.update(-configParam.Max_PWM, configParam.Max_PWM,
//                              configParam.p_M4.K_p, configParam.p_M4.K_i,configParam.p_M4.K_d);
//        }break;
//    }
//}
//
//
//mDeb_p.M1.Pwm_Out  =  motor1_pid.compute(mDeb_p.M1.Expectations,mDeb_p.M1.Feedback);
//mDeb_p.M2.Pwm_Out  =  motor2_pid.compute(mDeb_p.M2.Expectations,mDeb_p.M2.Feedback);
//mDeb_p.M3.Pwm_Out  =  motor3_pid.compute(mDeb_p.M3.Expectations,mDeb_p.M3.Feedback);
//mDeb_p.M4.Pwm_Out  =  motor4_pid.compute(mDeb_p.M4.Expectations,mDeb_p.M4.Feedback);
