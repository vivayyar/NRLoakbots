#include "NRL.h"
#include <math.h>
#include <HexaOLED.h>

// ---------- Controller layout (matches our labelled controller photo) ----------
// Our photo is taken from the far side, so it is upside down compared
// with the official HEXA "Button Labelling" diagram. Names below are the
// official ones. Change a name here to move a control - nothing else.
#define BTN_ARM_UP        BTN_DPAD_UP     // D-pad up    (photo: lower button of left cluster)
#define BTN_ARM_DOWN      BTN_DPAD_DOWN   // D-pad down  (photo: upper button of left cluster)
#define BTN_ECO_TOGGLE    BTN_DPAD_LEFT   // D-pad left  (photo: right button of left cluster)
#define BTN_GRIP_OPEN     BTN_RB          // RB          (photo: left red side button)
#define BTN_GRIP_CLOSE    BTN_LB          // LB          (photo: right red side button)
#define SW_SPORT          BTN_RT          // RIGHT_TOGGLE (photo: bottom-left switch) - check name!

// ---------- Hardware ----------
// FIX 1: flipped moved from the right motor to the left motor.
// Forward/back and left/right were all reversed, which means both
// motors were spinning the wrong way. Swapping the flip fixes both.
// minDrive = motor "stiction floor" (NRL HexaDCMotor.h): small commands are
// lifted to at least this PWM so the wheels actually turn instead of buzzing.
// NRL measured about 30/255 on their robot. Helps Eco push up slopes.
static HexaDCMotor leftMotor  {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true, .minDrive = 30 }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .minDrive = 30 }};
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
    // D-pad left toggles ECO on/off. Otherwise the right toggle
    // switch picks SPORT (up) or NORMAL (down).
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
    const DriveMode ECO    = { "ECO",    0.65f, 0.010f,  0.03f,  0.010f,  0.03f  }; // ~1 s to full, 65% speed (more push for slopes)
    const DriveMode NORMAL = { "NORMAL", 1.00f, 0.02f,   0.04f,  0.04f,   0.07f  }; // ~0.5 s to full, turns ~0.25 s
    const DriveMode SPORT  = { "SPORT",  1.00f, 0.035f,  0.06f,  0.035f,  0.06f  }; // ~0.3 s to full
    const DriveMode* mode = &NORMAL;   // start in NORMAL
    bool _eco = false;                 // eco on/off (toggled by its button)

    // ON-ROBOT OLED (Module 3.8)
    HexaOLED _oled{ hexaOledConfig() };
    bool _oledOk = false;
    int  _oledTick = 0;                // only redraw every 10 loops (10x/sec)

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
    // Turning uses a gentler curve, so small stick moves still turn promptly.
    static float turnCurve(float x) { return x * (0.5f + 0.5f * fabsf(x)); }

    // FIX 3: arm moves smoothly while D-pad up/down is HELD.
    float armTarget = 90;          // where the arm is being told to go
    const float ARM_STEP = 0.6f;   // degrees per tick (60 deg/sec) - slower = smoother under load
    // Full servo range per NRL HexaServo.h (500-2500us = 0-180 deg).
    // Arm is mounted reversed: ARM_MIN = arm fully UP, ARM_MAX = fully DOWN.
    // If the arm hits the frame at either end, pull that number back a little.
    const float ARM_MIN  = 0;      // arm fully up
    const float ARM_MAX  = 180;    // arm fully down

    // Gripper moves smoothly while RB/LB is HELD.
    float gripTarget = 90;          // where the gripper is being told to go
    const float GRIP_STEP = 1.5f;   // degrees per tick (150 deg/sec)
    const float GRIP_MIN  = 0;      // most closed - TEMPORARY 0 for calibration, see notes
    const float GRIP_MAX  = 120;    // most open   - adjust for your gripper

public:
    void init() override {
        // Runs once when INIT is pressed
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        gripper.begin();
        _oledOk = _oled.begin();       // remember whether the screen started
        armTarget = 90;
        gripTarget = 90;
        forwardNow = 0;
        turnNow = 0;
        mode = &NORMAL;
        _eco = false;
    }

    void loop() override {
        // Runs 100 times a second until STOP

        // ---- Drive mode ----
        // Eco button toggles eco on/off. Eco overrides the switch.
        // Otherwise the right toggle switch picks SPORT (up) or NORMAL (down).
        if (gamepad1.justPressed(BTN_ECO_TOGGLE)) _eco = !_eco;
        if (_eco)                            mode = &ECO;
        else if (gamepad1.pressed(SW_SPORT)) mode = &SPORT;
        else                                 mode = &NORMAL;

        // ---- Driving ----
        // Left stick = forward/back, right stick = turn
        // 1. Read the sticks and soften small movements
        float forwardWanted = curve(gamepad1.leftY());
        float turnWanted    = turnCurve(gamepad1.rightX());

        // 2. Ease toward what the sticks want, at the current mode's rate
        forwardNow = rampToward(forwardNow, forwardWanted, mode->fwdAccel,  mode->fwdDecel);
        turnNow    = rampToward(turnNow,    turnWanted,    mode->turnAccel, mode->turnDecel);

        // 3. Straight-line trim, mode's top speed, then drive
        float turn = turnNow - DRIVE_TRIM * forwardNow;
        drive.setScale(mode->topSpeed);
        drive.drive(forwardNow, turn);

        // ---- Arm ----
        // Hold to move. Servo is mounted reversed, so up = lower angle.
        if (gamepad1.pressed(BTN_ARM_UP))   armTarget -= ARM_STEP;
        if (gamepad1.pressed(BTN_ARM_DOWN)) armTarget += ARM_STEP;
        if (armTarget > ARM_MAX) armTarget = ARM_MAX;
        if (armTarget < ARM_MIN) armTarget = ARM_MIN;
        arm.setPosition(armTarget);

        // ---- Gripper ----
        // Hold RB to open, LB to close
        if (gamepad1.pressed(BTN_GRIP_OPEN))  gripTarget += GRIP_STEP;
        if (gamepad1.pressed(BTN_GRIP_CLOSE)) gripTarget -= GRIP_STEP;
        if (gripTarget > GRIP_MAX) gripTarget = GRIP_MAX;
        if (gripTarget < GRIP_MIN) gripTarget = GRIP_MIN;
        gripper.setPosition(gripTarget);

        // ---- Heading (IMU) ----
        // Press A to reset heading to 0. Keep robot still ~0.5 s after START.
        if (gamepad1.justPressed(BTN_A)) NRLComms::zeroHeading();

        // ---- Status LED (Module 3.7) ----
        // Low battery = fast red blink. Otherwise colour shows the drive mode.
        if (power.isBatteryLow())  userLed.setBlink(255, 0, 0, 250);
        else if (mode == &SPORT)   userLed.setSolid(80, 0, 0);    // red
        else if (mode == &ECO)     userLed.setSolid(0, 80, 0);    // green
        else                       userLed.setSolid(0, 0, 80);    // blue = NORMAL

        // ---- On-robot OLED (Module 3.8) ----
        if (_oledOk && ++_oledTick >= 10) {
            _oledTick = 0;
            _oled.clearDisplay();                    // wipe the old frame
            _oled.setTextSize(1);
            _oled.setTextColor(HexaOLED::WHITE);
            _oled.setCursor(0, 0);
            _oled.print(mode->name);
            _oled.print("  BAT ");
            _oled.print(power.getBatteryVoltage(), 1);
            _oled.print("V");
            _oled.setCursor(0, 12);
            _oled.print("HDG: ");
            if (NRLComms::isHeadingReady()) _oled.print(NRLComms::getHeading(), 1);
            else                            _oled.print("calib...");
            _oled.setCursor(0, 24);
            _oled.print("ARM: ");
            _oled.print(armTarget, 0);
            _oled.print("  GRIP: ");
            _oled.print(gripTarget, 0);
            _oled.display();                         // push it to the screen
        }

        // ---- Screen info (telemetry) ----
        // Rules: key max 11 chars, value max 15 chars.
        telemetry.addData("mode", mode->name);
        telemetry.addData("heading", NRLComms::isHeadingReady() ? NRLComms::getHeading() : -1.0f);
        telemetry.addData("hdg status", NRLComms::isHeadingReady() ? "READY" : "CALIBRATING");
        telemetry.addData("battery V", power.getBatteryVoltage());
        telemetry.addData("servo A", power.getServoCurrent());   // jumps up when a servo strains
        telemetry.addData("arm", armTarget);
        telemetry.addData("gripper", gripTarget);
    }

    void stop() override {
        // Runs once when STOP is pressed
        drive.stop();
        userLed.setOff();
        arm.detach();
        gripper.detach();
    }
};

REGISTER_OPMODE(Oakbots, "Oakbots", TELEOP);
