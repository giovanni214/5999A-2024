#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

//Defines controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Creates a motor group with forwards
pros::MotorGroup left_mg({-1, -2, -3}, pros::MotorGearset::blue); 
pros::MotorGroup right_mg({11, 12, 13}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&left_mg,                   // left motor group
               &right_mg,                  // right motor group
               13,                         // 13 inch track width
               lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
               450,                        // drivetrain rpm is 450
               2                           // horizontal drift is 2 (for now)
    );

// create the imu
pros::Imu imu(20);

// horizontal tracking wheel encoder
pros::adi::Encoder horizontal_encoder('A', 'B');

// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D');

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275, 2);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_275, 2);

//setup the full tracking sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as there isn't anither
    &imu     // inertial sensor
);

// lateral (moving) PID controller
lemlib::ControllerSettings
    lateral_controller(4.5,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,   // derivative gain (kD)
                       0,   // anti windup
                       0,   // small error range, in inches
                       0, // small error range timeout, in milliseconds
                       0,   // large error range, in inches
                       0, // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// angular (turning) PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                      13.8,  // derivative gain (kD)
                       0,   // anti windup
                       0,   // small error range, in degrees
                       0, // small error range timeout, in milliseconds
                       0,   // large error range, in degrees
                       0, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

pros::adi::Pneumatics mogoClamp('H', false);
pros::Motor lift_motor(7, pros::v5::MotorGears::blue);