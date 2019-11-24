#ifndef EXA_NUCLEAR_ROBOT_MOTOR_CONTROLLER
#define EXA_NUCLEAR_ROBOT_MOTOR_CONTROLLER

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "control_manager/control_manager.h"
#include "geometry_msgs/Twist"

#include <diff_drive_controller/diff_drive_controller.h>

namespace exa_controller
{
    class diff_drive_control
    {
    private:
        float right_wheel, left_wheel, wheel_radius, wheel_separation;
        double  gear_ratio = 11,
                wheel_diameter = 10, // in milli
                wheel_distance = 1, // wheel distance to other wheel
                rear_front_distace = 1,
                wheel_multiplier = 1;
                // note if value is 1 means it default value
            
            /*
                exa limitation info
            */
    public:
        diff_drive_control();
        ~diff_drive_control();
        void update();
        void init();

    }
}

#endif