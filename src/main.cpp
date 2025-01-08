#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <sys/_intsup.h>
#include <sys/types.h>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup
    left_mg({-1, -2, -3},
            pros::MotorGearset::blue); // Creates a motor group with forwards
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

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       10   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       12,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       2    // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

pros::adi::Pneumatics mogoClamp('H', false);
pros::Motor lift_motor(7, pros::v5::MotorGears::blue);
bool isRed = false;
bool hasExtraMogo = false;

void switchTeams() { isRed = !isRed; }

void switchSide() { hasExtraMogo = !hasExtraMogo; }

// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors

  pros::lcd::register_btn0_cb(switchTeams);
  pros::lcd::register_btn1_cb(switchSide);

  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      if (isRed)
        pros::lcd::print(3, "Team Color: RED SIDE");
      else
        pros::lcd::print(3, "Team Color: BLUE SIDE");

      if (hasExtraMogo)
        pros::lcd::print(4, "On Extra Mogo Side");
      else
        pros::lcd::print(4, "NOT On Extra Mogo Side");

      // delay to save resources
      pros::delay(20);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  // TODO
  if (isRed && hasExtraMogo) {

  } else if (isRed && !hasExtraMogo) {

  } else if (!isRed && hasExtraMogo) {

  } else if (!isRed && !hasExtraMogo) {
  }
}

void opcontrol() {
  bool isReversed = false;

  while (true) {
    // get left y and right x positions from controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    int isL1Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
    int isL2Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
    int isR1Pressed =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    int isR2Pressed =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    // when pressed, reverse the direction of the drive
    if (isL2Pressed) {
      isReversed = !isReversed;
    }

    if (!isReversed)
      chassis.tank(leftY, rightY);
    else
      chassis.tank(-rightY, -leftY);

    // toggle the mobile clamp when pressedd
    if (isL1Pressed)
      mogoClamp.toggle();

    // move lift and intake at full speed when btn pressed
    lift_motor.move(127 * (isR2Pressed - isR1Pressed));
    pros::delay(20); // Run for 20 ms then update
  }
}