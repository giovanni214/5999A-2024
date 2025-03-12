#include "auton.h"
#include "config.h"
#include "helper.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

void autonSkills() {
  // facing the alliance stake, (0, 0) is center
  chassis.setPose(-0.25, -53.5, 0);

  // score alliance ring
  lift_motor.move(127);
  pros::delay(500);
  lift_motor.brake();

  // move forward and face mogo
  chassis.moveToPose(-0.25, -44, 0, 3000, {}, false);
  chassis.turnToHeading(90, 1000, {.maxSpeed = 50}, false);

  // grab mogo
  chassis.moveToPose(-25, -44, 90, 2000, {.forwards = false, .minSpeed = 60},
                     false);
  mogoClamp.extend();

  pros::delay(1000);

  // spin up hook and intake
  intake_motor.move(65);
  lift_motor.move(127);

  // grab the first ring
  chassis.turnToHeading(0, 750, {.maxSpeed = 60}, false);
  chassis.moveToPose(-24, -18, 0, 2000, {.minSpeed = 50}, false);
  pros::delay(500);

  // grab second ring
  chassis.turnToHeading(-90, 750, {}, false);
  chassis.moveToPose(-48, -18, -90, 1000, {.minSpeed = 80}, false);
  pros::delay(1000);

  // grab third ring
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPose(-44, -48, 180, 1500, {.minSpeed = 80}, false);
  pros::delay(1000);

  // grab fourth ring
  chassis.moveToPose(-44, -60, 180, 1000, {.minSpeed = 80}, false);
  pros::delay(750);

  // back up to prepare for final ring grab
  chassis.moveToPose(-44, -24, 180, 1500, {.forwards = false}, false);

  // drive and grab last ring
  chassis.turnToPoint(-58, -50, 500, {},
                      false); // angled directly to last ring
  chassis.moveToPose(-58, -53, 206.5, 1000, {.minSpeed = 65}, false);
  pros::delay(500);

  chassis.moveToPose(-50, -40, 206.5, 1000, {.forwards = false}, false);
  chassis.turnToHeading(
      0, 2000,
      {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 50},
      false);

  chassis.moveToPose(-50, -54, -45, 1500, {.forwards = false, .minSpeed = 60},
                     false);

  // drop off mogo in corner
  mogoClamp.toggle();
  lift_motor.brake();
  intake_motor.brake();

  chassis.moveToPose(-48, -30, 0, 1000, {.maxSpeed = 50}, false);

  // reset x position
  chassis.moveToPose(-48, -48, -90, 3000, {}, false);
  chassis.moveToPose(-10000, -48, -90, 750, {}, false);
  pros::delay(500);
  chassis.setPose(-60, chassis.getPose().y, -90);

  // reset y position
  chassis.moveToPose(-36, chassis.getPose().y, -90, 3000, {.forwards = false});
  chassis.turnToHeading(0, 1000, {}, false);
  chassis.moveToPose(-36, -10000, 0, 750, {.forwards = false}, false);
  pros::delay(500);
  chassis.setPose(chassis.getPose().x, -62, 0);

  // get self ready to pick up next mogo
  chassis.moveToPose(-36, -48, 0, 1000, {}, false);
  chassis.turnToHeading(-90, 1000, {}, false);

  // grab second mogo
  chassis.moveToPose(-24, -48, -90, 500, {.forwards = false}, false);
  chassis.moveToPose(0, -48, -90, 500, {.forwards = false}, false);
  chassis.moveToPose(12, -48, -90, 500, {.forwards = false}, false);
  chassis.moveToPose(30, -48, -90, 1500, {.forwards = false, .minSpeed = 70},
                     false);
  pros::delay(500);
  mogoClamp.extend();
  pros::delay(500);

  intake_motor.move(65);
  lift_motor.move(127);

  // grab first ring (other side)
  chassis.turnToHeading(0, 1500, {}, false);
  chassis.moveToPose(24, -26, 0, 2000, {.minSpeed = 60}, false);
  pros::delay(1000);

  // grab second ring (other side)
  chassis.turnToHeading(90, 1500, {}, false);
  chassis.moveToPose(48, -26, 90, 2000, {.minSpeed = 80}, false);
  pros::delay(500);

  // grab third ring (other side)
  chassis.turnToHeading(180, 1500, {.maxSpeed = 60}, false);
  chassis.moveToPose(47, -48, 180, 2000, {.minSpeed = 80}, false);
  pros::delay(1500);

  // grab fourth ring (other side)
  chassis.moveToPose(47, -62, 180, 2000, {.minSpeed = 80}, false);
  pros::delay(1000);

  // back up from wall
  chassis.moveToPose(47, -54, 180, 500, {.forwards = false}, false);

  // turn to put other mogo back into corner
  chassis.turnToHeading(
      -45, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE},
      false);

  // go and drop mogo
  chassis.moveToPose(62, -62, -45, 2000, {.maxSpeed = 50}, false);
  mogoClamp.toggle();
}

void autonomousRoutine() {
  ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

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