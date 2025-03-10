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
bool hasExtraMogo = true;
bool isSkills = false;

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
    int vibrations = 0;
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
      if (lift_motor.get_temperature() > 40 && vibrations < 10) {
        controller.rumble(".");
        vibrations++;
      }

      // delay to save resources
      pros::delay(20);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void resetLadyBrown() {
  ladyBrownMotor.move(-127);
  pros::delay(1000);
  ladyBrownMotor.brake();
  ladyBrownRotation.reset_position();
}

void goToAngle(float angle) {
  ladyBrownMotor.move(160);
  while (getAngle() < angle) {
    pros::delay(5);
  }

  ladyBrownMotor.brake();
}

void autonSkills() {
  // facing the alliance stake, (0, 0) is center
  chassis.setPose(0, -57, 180);

  // score alliance ring
  lift_motor.move(127);
  pros::delay(250);
  lift_motor.brake();

  // go back to grab MOGO
  chassis.moveToPose(0, -48, 180, 500, {}, false);
  chassis.turnToHeading(-90, 1000, {}, false);
  chassis.moveToPoint(24, -48, 1000, {.forwards = false}, false);
  mogoClamp.toggle(); // grab mogo

  intake_motor.move(127);
  lift_motor.move(127);

  // grab ring above MOGO
  chassis.turnToHeading(0, 500, {}, false);
  chassis.moveToPoint(24, -24, 1000, {.forwards = true}, false);

  // turn to score red ring
  chassis.turnToPoint(48, 24, 1000, {}, false);
  chassis.moveToPoint(48, 24, 2000, {.forwards = true}, false);

  // back up infront of MOGO
  chassis.turnToPoint(42, 0, 1000, {}, false);
  chassis.moveToPoint(42, 0, 1000, {.forwards = false}, false);

  // score the ring into wall stake (GOOD LUCK LMAOOOO)
  chassis.turnToHeading(90, 1000, {}, false);
  chassis.moveToPoint(60, 0, 1000, {.forwards = true}, true);
  goToAngle(60);
  pros::delay(250);
  lift_motor.brake();
  chassis.moveToPoint(68, 0, 500, {}, true);
  goToAngle(120);
  pros::delay(250);
  resetLadyBrown();

  // Get next 4 rings in corner
  chassis.moveToPoint(48, 0, 1000, {.forwards = false}, false);
  chassis.turnToHeading(180, 500, {}, false);

  // grab next 3 rings
  chassis.moveToPoint(48, -24, 1000, {.forwards = false}, false);
  chassis.moveToPoint(48, -48, 1000, {.forwards = false}, false);
  chassis.moveToPoint(48, -60, 1000, {.forwards = false}, false);

  // turn to grab last ring
  chassis.turnToPoint(60, -48, 1000, {}, false);
  chassis.moveToPoint(60, -48, 500, {}, false);

  // drop MOGO in corner
  intake_motor.brake();
  chassis.turnToPoint(72, -72, 500, {.forwards = false}, false);
  chassis.moveToPoint(72, -72, 1000, {.earlyExitRange = 6}, false);
  mogoClamp.toggle();

  // get last ring and score into lady brown
  intake_motor.move(127);
  lift_motor.move(127);
  chassis.turnToPoint(48, 48, 500, {}, false);
  chassis.moveToPoint(48, 48, 5000, {.maxSpeed = 50}, false);

  // stop intake when ring is at the top
  while (getRingColor(optical_sensor) < 0) {
    pros::delay(20);
  }
  intake_motor.brake();
  lift_motor.brake();

  // doinker extra rings out of the way
  chassis.turnToHeading(-45, 500, {}, false);
  doinker.toggle();
  chassis.turnToHeading(90, 750, {}, false);
  chassis.turnToPoint(24, 60, 1000, {.forwards = false}, false);
  doinker.toggle();

  // grab blue mogo
  chassis.moveToPoint(24, 60, 3000, {}, false);
  mogoClamp.toggle(); // grab MOGO

  // put mogo in corner
  chassis.turnToHeading(90, 1000, {}, false);
  chassis.moveToPoint(60, 60, 3000, {}, false);
  chassis.turnToHeading(0, 1000, {}, false);
  chassis.moveToPoint(60, 70, 1000, {}, false);
  doinker.toggle();
  chassis.turnToPoint(72, 72, 1000, {}, false);
  chassis.moveToPoint(72, 72, 2000, {.forwards = false, .earlyExitRange = 6},
                      false);
  doinker.toggle();
  mogoClamp.toggle();

  
}

void autonomous() {
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  // TODO

  if (isSkills) {
    autonSkills();
  } else if (isRed && hasExtraMogo) {
    // preload red ring, put robot right behind mogo
    //  Get mogo and score a ring
    chassis.setPose(0, 12, 180);
    chassis.moveToPose(0, 48, 180, 3000, {.forwards = false}, false);
    chassis.moveToPose(0, 52, 180, 1000, {.forwards = false}, false);
    mogoClamp.toggle(); // grab
    lift_motor.move(127);

    chassis.moveToPose(20, 48, -90, 2000, {}, false);
    pros::delay(2000);
    chassis.turnToHeading(-90, 2000, {}, false);
    mogoClamp.toggle(); // let go
    chassis.moveToPose(-24, 48, -90, 3000, {}, false);
  } else if (!isRed && hasExtraMogo) {
    // preload blue ring, put robot right behind mogo
    chassis.setPose(0, 12, 180);
    chassis.moveToPose(0, 48, 180, 3000, {.forwards = false}, true);
    chassis.moveToPose(0, 52, 180, 1000, {.forwards = false}, true);
    mogoClamp.toggle(); // grab

    lift_motor.move(127);
    chassis.moveToPose(-28, 48, -90, 2000, {}, false);
    pros::delay(2000);
    chassis.turnToHeading(90, 2000);
    mogoClamp.toggle(); // let go
    chassis.moveToPose(24, 48, 90, 3000, {}, true);
  } else if (isRed && !hasExtraMogo) {
    // preload red ring, put robot right behind mogo
    chassis.setPose(0, 12, 180);
    chassis.moveToPose(0, 48, 180, 3000, {.forwards = false}, true);
    chassis.moveToPose(0, 52, 180, 1000, {.forwards = false}, true);
    mogoClamp.toggle(); // grab

    lift_motor.move(127);
    chassis.moveToPose(-28, 48, -90, 2000, {}, false);
    pros::delay(2000);
    chassis.turnToHeading(90, 2000);
    mogoClamp.toggle(); // let go
  } else if (!isRed && !hasExtraMogo) {
    // preload blue ring, put robot right behind mogo
    chassis.setPose(0, 12, 180);
    chassis.moveToPose(0, 48, 180, 3000, {.forwards = false}, false);
    chassis.moveToPose(0, 52, 180, 1000, {.forwards = false}, false);
    mogoClamp.toggle(); // grab
    lift_motor.move(127);

    chassis.moveToPose(20, 48, -90, 2000, {}, false);
    pros::delay(2000);
    chassis.turnToHeading(-90, 2000, {}, false);
    mogoClamp.toggle(); // let go
    chassis.moveToPose(-24, 48, -90, 3000, {}, false);
  }
}

void opcontrol() {
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  bool isReversed = false;

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

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
      doinker.toggle();

    // move lift based off of right triggers
    lift_motor.move(127 * (isR2Pressed - isR1Pressed));
    intake_motor.move(127 * (isR2Pressed - isR1Pressed));

    // Control for Lady Brown mech, could change if a better one is found
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      ladyBrownMotor.move(80);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      ladyBrownMotor.move(-30);
    } else {
      ladyBrownMotor.brake();
    }

    if (isAPressed) {
      isArmingLadyBrown = true;
      ladyBrownMotor.move(-127);
      pros::delay(250);
      ladyBrownRotation.reset_position();
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      ladyBrownMotor.move(-127);
    }

    if (isArmingLadyBrown) {
      // Angle varies, need to add mini angle adjustor
      if (getAngle() <= 60)
        ladyBrownMotor.move(40);
      else {
        ladyBrownMotor.brake();
        isArmingLadyBrown = false;
      }
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