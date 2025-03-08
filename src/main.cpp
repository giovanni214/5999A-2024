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

// void autonSkills() {
//   chassis.calibrate(true);
//   ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//   goToAngle(210);
//   resetLadyBrown();

//   // facing the alliance stake, (0, 0) is center
//   chassis.setPose(0, -60, 180);

//   // grab mogo
//   chassis.moveToPose(
//       -20, -52, 90, 10000,
//       {.forwards = false, .lead = 0.8, .minSpeed = 50, .earlyExitRange = 1},
//       false);

//   pros::delay(1000);

//   chassis.moveToPose(-36, -50, 90, 3000, {.forwards = false, .maxSpeed = 50},
//                      false);
//   mogoClamp.retract();

//   // Grab first ring
//   lift_motor.move(127);
//   chassis.moveToPose(-24, -28, 0, 3000, {}, false);

//   // Grab second ring
//   chassis.moveToPose(-50, -28, -90, 3000, {}, false);

//   // Third and fouth ring
//   chassis.moveToPose(-48, -50, 180, 5000, {.maxSpeed = 40}, false);
//   pros::delay(2000);
//   chassis.moveToPose(-48, -58, 180, 1000, {.maxSpeed=60}, false);

//   pros::delay(500);
//   // back up and grab final ring
//   chassis.moveToPose(-42, -24, 180, 3000, {.forwards = false, .maxSpeed=50},
//   false); chassis.moveToPose(-60, -48, 180, 3000, {.maxSpeed=50}, false);

//   pros::delay(1000);
//   // drop mobile goal off in corner
//   chassis.moveToPose(-65, -73, 45, 3000, {.forwards = false, .maxSpeed=40},
//   false); mogoClamp.extend(); lift_motor.brake();

//   //works perfect above

//   // back up, slam into wall and reset X
//   chassis.moveToPose(-44, -44, -90, 3000, {.maxSpeed=50}, false);
//   // chassis.moveToPose(-1000, -48, 90, 2000, {.forwards = false}, false);
//   // chassis.setPose(-62, chassis.getPose().y, 90);

//   pros::delay(1000);

//   // // stop, slam into wall to reset y
//   // chassis.moveToPose(-24, -55, -90, 5000, {.forwards = false}, false);
//   // chassis.moveToPose(-24, -1000, 180, 5000, {}, false);
//   // chassis.setPose(chassis.getPose().x + 2, chassis.getPose().y - 4, 90);

//   // go back and try to grab mogo
//   chassis.moveToPose(-24, -55, -90, 5000, {.forwards = false}, false);
//   chassis.moveToPose(0, -55, -90, 5000, {.forwards = false}, false);
//   chassis.moveToPose(24, -55, -90, 2000, {.forwards = false}, false);
//   chassis.moveToPose(30, -55, -90, 2000, {.forwards = false, .maxSpeed=80},
//   false); mogoClamp.retract();

//   lift_motor.move(127);
//   // grab first ring
//   chassis.moveToPose(48, -55, 90, 3000, {.maxSpeed=40}, false);

//   // grab second ring
//   //  chassis.moveToPose(48, -63, 90, 2000, {}, false);

//   // grab third ring
//   chassis.moveToPose(55, -55, 90, 3000, {.maxSpeed=55}, false);

//   // grab fourth one
//   chassis.moveToPose(20, -27, -45, 3000, {.maxSpeed=60}, false);

//   pros::delay(1000);

//   // drop off mogo
//   chassis.moveToPose(70, -72, -45, 5000, {.forwards = false}, false);
//   mogoClamp.extend();
//   lift_motor.brake();

//   pros::delay(1000);

//   chassis.moveToPose(48, 0, 0, 2000, {}, false);

//   chassis.moveToPose(48, 24, -45, 5000, {}, false);
//   chassis.moveToPose(24, 60, 90, 3000, {}, false);
//   chassis.moveToPose(72, 72, 90, 3000, {}, false);

//   chassis.moveToPose(60, 60, 45, 3000, {.forwards = false}, false);
// }

void autonSkills() {
  // facing the alliance stake, (0, 0) is center
  chassis.setPose(0, -57, 180);

  // score alliance ring
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  goToAngle(210);
  resetLadyBrown();

  // drive in front of mogo
  chassis.moveToPose(0, -48, 180, 3000, {.forwards = false}, false);
  chassis.turnToHeading(90, 2000, {.maxSpeed = 50}, false);

  // grab the mogo
  mogoClamp.extend();
  chassis.moveToPose(-24, -48, 90, 1000, {.forwards = false}, false);
  mogoClamp.toggle();

  // start intake
  lift_motor.move(127);

  // grab first ring
  chassis.turnToHeading(0, 1000, {}, false);
  chassis.moveToPose(-24, -24, 0, 2000, {}, false);

  // grab second ring
  chassis.turnToHeading(-90, 1000, {}, false);
  chassis.moveToPose(-45, -24, -90, 3000, {}, false);

  // grab third ring
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPose(-48, -45, 180, 3000, {}, false);

  pros::delay(1000);

  // grab fourth ring
  chassis.moveToPose(-48, -60, 180, 2000, {}, false);

  // back up to prepare for final ring grab
  chassis.moveToPose(-48, -24, 180, 3000, {.forwards = false}, false);

  // drive and grab last ring
  chassis.turnToHeading(206.5, 1000, {}, false); // angled directly to last ring
  chassis.moveToPose(-60, -48, 206.5, 3000, {}, false);

  // turn around and to drop mogo off
  chassis.turnToHeading(45, 1000, {}, false);

  // drop off mogo in corner
  chassis.moveToPose(-65, -65, 45, 3000, {.forwards = false}, false);
  mogoClamp.toggle();
  lift_motor.brake();

  // get out of the way and turn towards other mogo
  chassis.moveToPose(-48, -48, 45, 3000, {}, false);
  chassis.turnToHeading(-90, 1000, {}, false);

  // drive to mogo
  chassis.moveToPose(-24, -48, 45, 1000, {}, false);
  chassis.moveToPose(0, -48, 45, 1000, {}, false);
  chassis.moveToPose(12, -48, 45, 1000, {}, false);
  chassis.moveToPose(24, -48, 45, 1000, {}, false);

  // grab mogo
  mogoClamp.toggle();
  lift_motor.move(127);

  // get 2nd mogo 1st ring
  chassis.turnToHeading(0, 1000, {}, false);
  chassis.moveToPose(24, -24, 0, 1000, {}, false);

  // get 2nd mogo 2nd ring
  chassis.turnToHeading(90, 1000, {}, false);
  chassis.moveToPose(48, -24, 90, 1000, {}, false);

  // get 2nd mogo 3rd ring
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPose(48, -45, 180, 3000, {}, false);

  pros::delay(1500);

  // grab fourth ring (2nd mogo)
  chassis.moveToPose(48, -60, 180, 2000, {}, false);

  // back up to prepare for final ring grab
  chassis.moveToPose(48, -24, 180, 3000, {.forwards = false}, false);

  // drive and grab last ring
  chassis.turnToHeading(153.4, 1000, {}, false); // angled directly to last ring
  chassis.moveToPose(60, -48, 153.4, 3000, {}, false);
  // // drive forward to get mobile goal
  // chassis.moveToPose(-48, -24, 180, 3000, {.forwards = false}, false);
  // chassis.moveToPose(-48, 0, 180, 3000, {.forwards = false}, false);
  // chassis.moveToPose(-48, 24, 180, 3000, {.forwards = false}, false);
  // chassis.moveToPose(-48, 36, 180, 3000, {.forwards = false}, false);
  // chassis.turnToHeading(225, 2000, {}, false);

  // // go and grab mobile goal
  // chassis.moveToPose(-24, 60, 225, 2000, {.forwards = false}, false);
  // mogoClamp.toggle();

  // // turn to push mobile goal into corner
  // chassis.turnToHeading(90, 1000, {}, false);

  // // drop off mogo
  // chassis.moveToPose(-80, 60, 90, 3000, {.forwards = false}, false);
  // mogoClamp.toggle();

  // // push other mogo in corner (in small steps to ensure accuracy)
  // chassis.moveToPose(-50, 60, 90, 3000, {.forwards = false}, false);
  // chassis.moveToPose(-30, 60, 90, 1000, {}, false);
  // chassis.moveToPose(0, 60, 90, 1000, {}, false);
  // chassis.moveToPose(30, 60, 90, 1000, {}, false);
  // chassis.moveToPose(60, 60, 90, 1000, {}, false);
  // chassis.moveToPose(80, 60, 90, 1000, {}, false);

  // // back up from corner
  //  chassis.moveToPose(0, 0, 180, 3000);
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