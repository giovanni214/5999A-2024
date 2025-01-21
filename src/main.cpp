#include "main.h"
#include "config.h"
#include "helper.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <sys/_intsup.h>
#include <sys/types.h>

bool isRed = true;
bool hasExtraMogo = false;
bool isSkills = true;

void switchTeams() { isRed = !isRed; }

void switchSide() { hasExtraMogo = !hasExtraMogo; }

void switchSkills() { isSkills = !isSkills; }

float getAngle() { return centiToDegrees(ladyBrownRotation.get_position()); }

// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors

  pros::lcd::register_btn0_cb(switchTeams);
  pros::lcd::register_btn1_cb(switchSide);
  pros::lcd::register_btn2_cb(switchSkills);

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

      if (!isSkills) {
        pros::lcd::print(3, "%s SIDE %s On Extra Mogo Side",
                         isRed ? "RED" : "BLUE", hasExtraMogo ? "ON" : "NOT");
      }

      pros::lcd::print(4, "%s MODE", isSkills ? "SKILLS" : "COMP");

      pros::lcd::print(5, "Angle %f", getAngle());

      pros::lcd::print(6, "TEMP: %f", lift_motor.get_temperature());
      if (lift_motor.get_temperature() > 40) {
        controller.rumble(".");
      }

      // delay to save resources
      pros::delay(20);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // TODO

  if (isSkills) {

  } else if (isRed && hasExtraMogo) {

  } else if (isRed && !hasExtraMogo) {
    // Do first. Move forward, put lady brown to score, then grab mogo
  } else if (!isRed && hasExtraMogo) {

  } else if (!isRed && !hasExtraMogo) {
  }
}

void opcontrol() {
  bool isReversed = false;
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  bool isArmingLadyBrown = false;
  int rejectBlueRingCount = 0;
  int lastColorSeen = -1;

  while (true) {
    // get left y and right x positions from controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    int isL1Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
    int isL2Pressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
    int isAPressed =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
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

    if (isAPressed)
      isArmingLadyBrown = true;

    if (isArmingLadyBrown) {
      // Angle varies, need to add mini angle adjustor
      if (getAngle() <= 58)
        ladyBrownMotor.move(30);
      else {
        ladyBrownMotor.brake();
        isArmingLadyBrown = false;
      }
    }

    // move lift based off of right triggers
    lift_motor.move(127 * (isR2Pressed - isR1Pressed));

    // Control for Lady Brown mech, could change if a better one is found
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      ladyBrownMotor.move(80);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      ladyBrownMotor.move(-30);
    } else {
      ladyBrownMotor.brake();
    }

    // quick, dirty way to reset Lady Brown
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      ladyBrownMotor.move(-127);
      pros::delay(1000);
      ladyBrownRotation.reset_position();
    }

    if (isSkills) {
      int ringColor = getRingColor(optical_sensor); // 1 is blue

      if (rejectBlueRingCount > 0) {
        rejectBlueRingCount += 20;
      }

      if (rejectBlueRingCount >= 250) {
        lift_motor.brake();
      }

      if (rejectBlueRingCount >= 500) {
        lift_motor.move(127); // restart lift
        rejectBlueRingCount = 0;
      }

      // If a blue ring has been seen, and we aren't counting, and we haven't
      // seen a blue last time
      if (ringColor == 1 && rejectBlueRingCount == 0 && lastColorSeen < 1) {
        rejectBlueRingCount += 20;
      }

      lastColorSeen = ringColor;
    }

    pros::delay(10); // Run for 10 ms then update
  }
}