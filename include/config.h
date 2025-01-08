#ifndef CONFIG_H
#define CONFIG_H

#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"

extern pros::Controller controller;
extern lemlib::Chassis chassis;

//the stuff I actually care about
extern pros::adi::Pneumatics mogoClamp;
extern pros::Motor lift_motor;


#endif // CONFIG_H