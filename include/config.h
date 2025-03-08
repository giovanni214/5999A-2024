#ifndef CONFIG_H
#define CONFIG_H

#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include "pros/optical.hpp"

extern pros::Controller controller;
extern lemlib::Chassis chassis;

//the stuff I actually care about
extern pros::adi::Pneumatics mogoClamp;
extern pros::adi::Pneumatics doinker;
extern pros::Motor intake_motor;
extern pros::Motor lift_motor;
extern pros::Rotation ladyBrownRotation;
extern pros::Motor ladyBrownMotor;
extern pros::Optical optical_sensor;



#endif // CONFIG_H