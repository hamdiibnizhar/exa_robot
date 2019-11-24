 #pragma once
//#include "exa_control.h"
#include "exa_motor_controller.h"

using namespace exa_controller;

exa_controller::diff_drive_control diff_drive_control(float r_wheel, float l_wheel)
{
    this->right_wheel = r_wheel;
    this->left_wheel = l_wheel;
} 

void twistCallack(const geometry_msgs::TwistConstPtr& twist_ptr)
{
    double vel_x = twist_ptr->linear.x;
    double vel_w = twist_ptr->linear.z;

    vel_x *= this->right_wheel;
    
}

void exa_controller::diff_drive_control update()
{

}