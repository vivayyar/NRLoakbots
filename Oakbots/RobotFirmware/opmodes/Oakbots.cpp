#include "NRL.h"
#include <math.h>


// ---------- Hardware ----------
// FIX 1: flipped moved from the right motor to the left motor.
// Forward/back and left/right were all reversed, which means both
// motors were spinning the wrong way. Swapping the flip fixes both.
static HexaDCMotor leftMotor  {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM }};
static TankDrive drive(leftMotor, rightMotor);
static HexaServo arm     {{ .signalPin = SERVO_1, .startAngle = 90.0f, .settleMs = 0 }};
static HexaServo gripper {{ .signalPin = SERVO_2, .startAngle = 90.0f, .settleMs = 0 }};


class Oakbots : public NRLOpMode {
    // FIX 2: straight-line trim.
    // The left wheels are faster, so the robot curves right.
    // This adds a small left correction whenever you drive.
    // Still curves right -> make it bigger (0.08, 0.10...)
    // Now curves left    -> make it smaller (0.03, 0.02...)
    const float DRIVE_TRIM = 0.05f;


    // DRIVE MODES
    // RB toggles SPORT <-> NORMAL.  LB toggles ECO <-> NORMAL.
    // The mode stays until you press again.
    //
    // topSpeed: 1.0 = full speed (255), 0.5 = half speed
    // accel/decel values: time from stop to full = 0.01 / value seconds
    //   (smaller = gentler, bigger = punchier)
    struct DriveMode {
        const char* name;
        float topSpeed;
        float fwdAccel, fwdDecel;
        float turnAccel, turnDecel;
    };
    //                         name      top    fwdAcc   fwdDec  turnAcc  turnDec
    const DriveMode ECO    = { "ECO",    0.50f, 0.007f,  0.02f,  0.008f,  0.025f }; // ~1.4 s to full, half speed
    const DriveMode NORMAL = { "NORMAL", 1.00f, 0.0125f, 0.025f, 0.015f,  0.03f  }; // ~0.8 s to full
    const DriveMode SPORT  = { "SPORT",  1.00f, 0.025f,  0.04f,  0.025f,  0.04f  }; // ~0.4 s to full
    const DriveMode* mode = &NORMAL;   // start in NORMAL


    float forwardNow = 0;              // the smoothed values actually sent
    float turnNow    = 0;


    // Moves 'current' toward 'target', but only by a small amount each tick.
    // Speeding up uses 'up', slowing down or reversing uses 'down'.
    static float rampToward(float current, float target, float up, float down) {
        bool speedingUp = (fabsf(target) > fabsf(current)) && (target * current >= 0);
        float step = speedingUp ? up : down;
        if (target > current) return fminf(current + step, target);
        if (target < current) return fmaxf(current - step, target);
        return current;
    }


    // Makes small stick movements gentler for fine control,
    // while full stick still gives full speed.
    static float curve(float x) { return x * fabsf(x); }


    // FIX 3: arm moves smoothly while D-pad is HELD.
    float armTarget = 90;          // where the arm is being told to go
    const float ARM_STEP = 1.0f;   // degrees per tick (100 ticks/sec = 100 deg/sec)
    const float ARM_MIN  = 20;     // lowest the arm may go  - adjust for your robot
    const float ARM_MAX  = 160;    // highest the arm may go - adjust for your robot


    // Gripper moves smoothly while D-pad left/right is HELD.
    float gripTarget = 90;          // where the gripper is being told to go
    const float GRIP_STEP = 1.5f;   // degrees per tick (150 deg/sec)
    const float GRIP_MIN  = 60;     // most closed - adjust for your gripper
    const float GRIP_MAX  = 120;    // most open   - adjust for your gripper


public:
    void init() override {
        // Runs once when INIT is pressed
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        gripper.begin();
        armTarget = 90;
        gripTarget = 90;
        forwardNow = 0;
        turnNow = 0;
        mode = &NORMAL;
    }


    void loop() override {
        // Runs 100 times a second until STOP


        // ---- Drive mode ----
        // RB: switch between SPORT and NORMAL
        if (gamepad1.justPressed(BTN_RB)) mode = (mode == &SPORT) ? &NORMAL : &SPORT;
        // LB: switch between ECO and NORMAL
        if (gamepad1.justPressed(BTN_LB)) mode = (mode == &ECO)   ? &NORMAL : &ECO;


        // ---- Driving ----
        // Left stick = forward/back, right stick = turn
        // 1. Read the sticks and soften small movements
        float forwardWanted = curve(gamepad1.leftY());
        float turnWanted    = curve(gamepad1.rightX());


        // 2. Ease toward what the sticks want, at the current mode's rate
        forwardNow = rampToward(forwardNow, forwardWanted, mode->fwdAccel,  mode->fwdDecel);
        turnNow    = rampToward(turnNow,    turnWanted,    mode->turnAccel, mode->turnDecel);


        // 3. Straight-line trim, mode's top speed, then drive
        float turn = turnNow - DRIVE_TRIM * forwardNow;
        drive.setScale(mode->topSpeed);
        drive.drive(forwardNow, turn);


        // ---- Arm ----
        // Hold D-pad up to raise, hold D-pad down to lower
        if (gamepad1.pressed(BTN_DPAD_UP))   armTarget -= ARM_STEP;   // servo is mounted reversed,
        if (gamepad1.pressed(BTN_DPAD_DOWN)) armTarget += ARM_STEP;   // so up = lower angle
        if (armTarget > ARM_MAX) armTarget = ARM_MAX;
        if (armTarget < ARM_MIN) armTarget = ARM_MIN;
        arm.setPosition(armTarget);


        // ---- Gripper ----
        // Hold D-pad left to open, hold D-pad right to close
        if (gamepad1.pressed(BTN_DPAD_LEFT))  gripTarget += GRIP_STEP;
        if (gamepad1.pressed(BTN_DPAD_RIGHT)) gripTarget -= GRIP_STEP;
        if (gripTarget > GRIP_MAX) gripTarget = GRIP_MAX;
        if (gripTarget < GRIP_MIN) gripTarget = GRIP_MIN;
        gripper.setPosition(gripTarget);


        // ---- Screen info (helps with testing) ----
        telemetry.addData("mode", mode->name);
        telemetry.addData("arm", armTarget);
        telemetry.addData("gripper", gripTarget);
        telemetry.addData("dpad up", gamepad1.pressed(BTN_DPAD_UP));
        telemetry.addData("dpad down", gamepad1.pressed(BTN_DPAD_DOWN));
    }


    void stop() override {
        // Runs once when STOP is pressed
        drive.stop();
        arm.detach();
        gripper.detach();
    }
};


REGISTER_OPMODE(Oakbots, "Oakbots", TELEOP);