#include "NRL.h"

// =====================================================================
//  Oakbots LINE AUTO - NRL 2026 HYPERDRIVE Autonomous Skill Challenge
// =====================================================================
//  Uses NRL's own LineActions (LineActions.h / LineTuning.h in the kit):
//  PID line following, junction counting, automatic corners, gyro turns,
//  U-turns, black-box START/PARK detection and fault handling.
//
//  BEFORE THE FIRST RUN: calibrate the ARC-8 on the controller:
//    CALIBRATE > LINE TEST  ->  step 1 all 8 sensors on WHITE,
//                               step 2 all 8 sensors on the BLACK line
//    (rotate the robot so the bar lies ALONG the line for step 2).
//  It is saved and survives power-off. Redo it at every new venue.
//
//  SET-UP: robot inside the START square, facing RIGHT along the bottom
//  line (towards the bottom-right corner), square to the line.
//  It drives the loop anticlockwise; every corner is a left turn and is
//  taken automatically (corners are never counted).
//
//  Facing the direction of travel, Pick-Up terminals are always on the
//  robot's RIGHT (outward) and Drop terminals on its LEFT (inward):
//     right side, going up:   P1(R)  D1(L)  P2(R)
//     top, going left:        P3(R)  D2(L)  P4(R)   <- both GOLD + a Drop
//     left side, going down:  P5(R)  D3(L)  P6(R)
// =====================================================================

// ---------- Hardware (same wiring as the TeleOp) ----------
static HexaDCMotor leftMotor  {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM }};
static TankDrive     drive(leftMotor, rightMotor);
static DriveActions  driveActions(drive);
static HexaLineArray line{ hexaLineConfig() };          // ARC-8 line sensor
static HexaServo arm     {{ .signalPin = SERVO_1, .startAngle = 60.0f, .settleMs = 0 }};
static HexaServo gripper {{ .signalPin = SERVO_2, .startAngle = 90.0f, .settleMs = 0 }};

// ---------- Choose the routine ----------
//   1 = Leave START, follow the loop, Park            (35 points)
//   2 = Stage 1 + both GOLD CORE delivered via D2      (95 points)
#define STAGE 1

// ---------- Mechanism positions - TUNE on the real robot ----------
// Arm servo is mounted reversed: LOWER angle = arm HIGHER.
static constexpr float ARM_CARRY   = 60;    // arm up, CORE held clear of the floor
static constexpr float ARM_FLOOR   = 150;   // arm down, gripper around the CORE
static constexpr float ARM_RELEASE = 100;   // arm part-way, to drop over the end bar
// NRL's own example gripper: 0 = closed, 180 = open.
static constexpr float GRIP_OPEN   = 120;
static constexpr float GRIP_CLOSED = 5;
static constexpr float GRAB_REACH_IN    = 2.0f; // drive on past the end bar to reach the CORE
static constexpr float RELEASE_REACH_IN = 1.0f; // drive on past a Drop end bar before release

// ---------- Line-following tuning (defaults are NRL's) ----------
static LineTuning oakTuning() {
    LineTuning t;
    t.followSpeed = 0.35f;    // forward speed on a straight - raise once it is reliable
    t.pid.kP      = 0.50f;    // steering strength: zig-zags -> lower, drifts off -> raise
    t.pid.kD      = 0.04f;    // damping after turns
    return t;
}
static LineActions lineActions(drive, line, oakTuning());

// ---------- Our own pick-up / drop sequences ----------
static ActionPtr grabCore() {
    return sequential({
        instant([]{ gripper.setPosition(GRIP_OPEN); }),
        instant([]{ arm.setPosition(ARM_FLOOR); }),
        sleep_ms(500),
        driveActions.driveInches(GRAB_REACH_IN),        // reach the CORE beyond the end bar
        instant([]{ gripper.setPosition(GRIP_CLOSED); }),
        sleep_ms(400),
        instant([]{ arm.setPosition(ARM_CARRY); }),      // lift so it is carried, not dragged
        sleep_ms(500),
        driveActions.driveInches(-GRAB_REACH_IN),       // back to the end bar
    });
}

static ActionPtr releaseCore() {
    return sequential({
        driveActions.driveInches(RELEASE_REACH_IN),     // CORE must land BEYOND the end bar
        instant([]{ arm.setPosition(ARM_RELEASE); }),
        sleep_ms(400),
        instant([]{ gripper.setPosition(GRIP_OPEN); }),
        sleep_ms(400),
        instant([]{ arm.setPosition(ARM_CARRY); }),
        sleep_ms(300),
        driveActions.driveInches(-RELEASE_REACH_IN),
    });
}

// ---------- Routes ----------
static ActionPtr stage1_leaveAndPark() {
    return lineActions.auto_()
        .leaveBox()          // drive out of the black START square (Leave Start: 5)
        .followToBox()       // follow the whole loop, corners turn themselves
        .park(400)           // on into the black PARK square, stop  (Park: 30)
        .build();
}

static ActionPtr stage2_goldRun() {
    return lineActions.auto_()
        .leaveBox()
        // ---- P3 (GOLD): 3rd terminal on the right (P1, P2, P3) ----
        .followToJunction(LineSide::RIGHT, 3)
        .right().followToEnd()
        .then(grabCore())
        .uTurn().followToJunction()      // back to the top line, facing in
        .right()                         // carry on along the top, heading left
        // ---- D2: next terminal on the left ----
        .followToJunction(LineSide::LEFT, 1)
        .left().followToEnd()
        .then(releaseCore())             // GOLD #1 delivered (25)
        .uTurn().followToJunction()      // back to the top line, facing out
        .left()                          // heading left again, towards P4
        // ---- P4 (GOLD) ----
        .followToJunction(LineSide::RIGHT, 1)
        .right().followToEnd()
        .then(grabCore())
        .uTurn().followToJunction()      // back to the top line, facing in
        .left()                          // U-turn route: head RIGHT, back to D2
        .followToJunction(LineSide::RIGHT, 1)   // D2 is now on our right
        .right().followToEnd()
        .then(releaseCore())             // GOLD #2 delivered (25)
        .uTurn().followToJunction()      // back to the top line, facing out
        .left()                          // heading left: past P4, down the left side
        // ---- Home ----
        .followToBox()
        .park(400)
        .build();
}

class OakbotsLine : public NRLOpMode {
public:
    // INIT: claim hardware only - nothing moves while the robot is lined up.
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        gripper.begin();
        line.begin();                    // loads the saved LINE TEST calibration
    }

    // START: queue the routine ONCE.
    void start() override {
        lineActions.clearRoute();        // this spot (square to the line) = grid 0
        lineActions.clearFault();
        lineActions.resetCounts();
        gripper.setPosition(GRIP_OPEN);
        arm.setPosition(ARM_CARRY);
        runAction(sequential({
            driveActions.waitForHeadingReady(),   // hold still ~0.5 s for the IMU
            (STAGE == 1) ? stage1_leaveAndPark() : stage2_goldRun(),
            instant([]{ drive.stop(); }),
        }));
    }

    // LOOP: only watch.
    void loop() override {
        telemetry.addData("calibrated", line.isCalibrated() ? "YES" : "NO - CAL!");
        telemetry.addData("fault",      lineActions.faultName());
        telemetry.addData("junctions",  lineActions.junctionCount());
        telemetry.addData("pattern",    linePatternName(line.pattern()));
        telemetry.addData("line pos",   line.position());
        telemetry.addData("battery V",  power.getBatteryVoltage());

        if (!line.isCalibrated())                    userLed.setBlink(255, 0, 255, 250); // purple: calibrate!
        else if (lineActions.fault() != LineFault::NONE) userLed.setSolid(255, 0, 0);    // red: fault
        else if (power.isBatteryLow())               userLed.setBlink(255, 0, 0, 250);
        else                                         userLed.setPulse(80, 40, 0, 1000);  // amber: running
    }

    // STOP: motors do not stop by themselves.
    void stop() override {
        drive.stop();
        userLed.setOff();
        arm.detach();
        gripper.detach();
    }
};

REGISTER_OPMODE(OakbotsLine, "Oakbots Line", AUTO);
