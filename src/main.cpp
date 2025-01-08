#include "main.h"
#include "config.h"
#include "helper.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <sys/_intsup.h>
#include <sys/types.h>

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

  optical_sensor.set_led_pwm(100);

  ladyBrownRotation.set_data_rate(5);
  ladyBrownRotation.set_reversed(true);
  ladyBrownRotation.reset_position();

  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      if (isRed)
        pros::lcd::print(3, "RED SIDE");
      else
        pros::lcd::print(3, "BLUE SIDE");

      if (hasExtraMogo)
        pros::lcd::print(4, "On Extra Mogo Side");
      else
        pros::lcd::print(4, "NOT On Extra Mogo Side");

      pros::lcd::print(6, "Angle: %f",
                       centiToDegrees(ladyBrownRotation.get_position()));

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

float getAngle() { return centiToDegrees(ladyBrownRotation.get_position()); }

void opcontrol() {
  bool isReversed = false;
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
    // get left y and right x positions from controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    int isL1Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
    int isL2Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
    int isR1Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    int isR2Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

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

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      ladyBrownMotor.move(30);
      while (getAngle() <= 47) {
        pros::delay(5);
      }

      ladyBrownMotor.brake();
    }

    if (getRingColor(optical_sensor) > -1 && getAngle() >= 47) {
      ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      while (getAngle() < 140) {
        ladyBrownMotor.move(60);
        pros::delay(50);
      }

      ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lift_motor.brake();
      ladyBrownMotor.brake();
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      ladyBrownMotor.move(30);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      ladyBrownMotor.move(-20);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      ladyBrownMotor.move(-70);
      pros::delay(1000);
      ladyBrownRotation.reset_position();
    }

    // move lift and intake at full speed when btn pressed
    lift_motor.move(127 * (isR2Pressed - isR1Pressed));
    pros::delay(20); // Run for 20 ms then update
  }
}