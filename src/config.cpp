#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// Defines controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Creates a motor group with forwards
//starting from back 
pros::MotorGroup left_mg({-8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({15, -3, 2}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&left_mg,                   // left motor group
               &right_mg,                  // right motor group
               13,                         // 13 inch track width
               lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
               450,                        // drivetrain rpm is 450
               2                           // horizontal drift is 2 (for now)
    );

// create the imu (inertial sensor)
pros::Imu imu(12);

// horizontal tracking wheel encoder
pros::adi::Encoder horizontal_encoder('C', 'D');

// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('A', 'B');

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275, 2);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_275, 2);

// setup the full tracking sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as there isn't
             // anither
    &imu     // inertial sensor
);

// lateral (moving) PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       1,   // small error range timeout, in milliseconds
                       100, // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular (turning) PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

pros::adi::Pneumatics mogoClamp('H', true);
pros::adi::Pneumatics doinker('F', false);
pros::Motor intake_motor(-20, pros::v5::MotorGears::green);
pros::Motor lift_motor(-7, pros::v5::MotorGears::blue);
pros::Rotation ladyBrownRotation(-18);
pros::Motor ladyBrownMotor(21, pros::MotorGearset::green);
pros::Optical optical_sensor(19);
