#include "auton.h"
#include "config.h"
#include "helper.h"
#include "pros/rtos.hpp"

void autonSkills() {
  // facing the alliance stake, (0, 0) is center
  chassis.setPose(0, -62.5, 0);

  // score alliance ring
  lift_motor.move(127);
  pros::delay(500);
  lift_motor.brake();

  // go back to grab MOGO
  chassis.moveToPose(0, -52, 0, 5000, {.maxSpeed = 30}, false);

  pros::delay(50);
  chassis.turnToHeading(-90, 1000, {}, false);
  pros::delay(50);
  chassis.moveToPoint(24, -52, 1000, {.forwards = false, .maxSpeed = 80},
                      false);
  mogoClamp.toggle(); // grab mogo

  pros::delay(500);
  intake_motor.move(127);
  lift_motor.move(127);

  // grab first ring
  chassis.turnToHeading(0, 500, {}, false);
  chassis.moveToPoint(24, -24, 3000, {.forwards = true}, false);

  pros::delay(1000);

  // turn to go around ladder and score red ring
  chassis.turnToPoint(48, 0, 1000, {}, false);
  chassis.moveToPoint(48, 0, 2000, {.forwards = true, .earlyExitRange = 4},
                      false);

  pros::delay(1000);

  // score red ring
  chassis.turnToPoint(48, 24, 3000, {.maxSpeed = 40}, false);
  chassis.moveToPoint(48, 24, 2000, {.forwards = true, .maxSpeed = 70}, false);

  pros::delay(1000);

  // back up infront of MOGO
  chassis.moveToPoint(42, -10, 1000, {.forwards = false, .maxSpeed = 40},
                      false);
  pros::delay(250);

  // score the ring into wall stake (GOOD LUCK LMAOOOO)
  chassis.turnToHeading(90, 3000, {}, false);
  chassis.moveToPoint(68, -10, 500, {}, true);
  goToAngle(30);
  pros::delay(1000);
  lift_motor.brake();
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

void autonomousRoutine() {
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