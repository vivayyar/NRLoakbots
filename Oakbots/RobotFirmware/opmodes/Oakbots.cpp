#include "NRL.h"
#include <math.h>
#include <HexaOLED.h>

// =====================================================================
//  Oakbots TeleOp - NRL 2026 HYPERDRIVE
// =====================================================================

// ---------- Controller layout (official HEXA names) ----------
// Our photo is taken from the far side, so it is upside down compared
// with the official "Button Labelling" diagram. Change a name here to
// move a control - nothing else needs editing.
#define BTN_ARM_UP        BTN_DPAD_UP     // hold: arm up
#define BTN_ARM_DOWN      BTN_DPAD_DOWN   // hold: arm down
#define BTN_DOCK          BTN_DPAD_LEFT   // HOLD: auto-dock / auto-balance
#define BTN_DROPPER       BTN_DPAD_RIGHT  // press: payload dropper open/close
#define BTN_GRIP_OPEN     BTN_RB          // hold: open gripper
#define BTN_GRIP_CLOSE    BTN_LB          // hold: close gripper
#define SW_ECO            BTN_RT          // right toggle switch: up = ECO, down = NORMAL

// ---------- Hardware ----------
// flipped on the LEFT motor: with it on the right, every control was reversed.
// minDrive (NRL HexaDCMotor.h "stiction floor"): small commands are lifted to
// at least this PWM so the wheels turn instead of buzzing - helps on slopes.
static HexaDCMotor leftMotor  {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true, .minDrive = 30 }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .minDrive = 30 }};
static TankDrive drive(leftMotor, rightMotor);
static HexaServo arm     {{ .signalPin = SERVO_1, .startAngle = 90.0f, .settleMs = 0 }};
static HexaServo gripper {{ .signalPin = SERVO_2, .startAngle = 90.0f, .settleMs = 0 }};

// Payload dropper at the back - servo in SERVO 3.
static constexpr float DROP_CLOSED = 0;     // holds the items in   - tune
static constexpr float DROP_OPEN   = 90;    // lets the items fall  - tune
static HexaServo dropper {{ .signalPin = SERVO_3, .startAngle = DROP_CLOSED, .settleMs = 0 }};

// Our own handle on the on-board IMU for TILT (Module 3.5). Heading still
// comes from NRLComms.
static HexaIMU imu{ hexaImuConfig() };

// ---------- Auto-dock tuning ----------
// HOW TO USE: drive FORWARD a little way up the slope, then HOLD D-pad left.
// The robot works out which way is uphill from the tilt it feels at that
// moment, drives forward, and settles where the dock is level.
static constexpr float DOCK_LOCK_DEG  = 4.0f;   // tilt needed to know which way is uphill
static constexpr float DOCK_SEEK      = 0.25f;  // creep forward if not tilted enough yet
static constexpr float BAL_KP         = 0.025f; // drive per degree of tilt
static constexpr float BAL_KD         = 0.004f; // damping per deg/s (stops rocking past level)
static constexpr float BAL_MIN        = 0.18f;  // smallest push that still climbs
static constexpr float BAL_MAX        = 0.40f;  // never faster than this while docking
static constexpr float BAL_LEVEL      = 2.5f;   // within +/- this = level: hold still
static constexpr float BAL_HDG_KP     = 0.02f;  // keeps the robot square on the dock

class Oakbots : public NRLOpMode {
    // ---------- Driving ----------
    // Straight-line trim: left wheels are faster, so add a small left correction.
    const float DRIVE_TRIM = 0.05f;

    // Two modes, picked by the right toggle switch.
    // accel/decel: time from stop to full = 0.01 / value seconds.
    struct DriveMode {
        const char* name;
        float topSpeed;
        float fwdAccel, fwdDecel;
        float turnAccel, turnDecel;
    };
    //                         name      top    fwdAcc  fwdDec turnAcc turnDec
    const DriveMode ECO    = { "ECO",    0.65f, 0.010f, 0.03f, 0.010f, 0.03f }; // ~1 s to full, 65% speed
    const DriveMode NORMAL = { "NORMAL", 1.00f, 0.020f, 0.04f, 0.040f, 0.07f }; // ~0.5 s to full, quick turns
    const DriveMode* mode = &NORMAL;

    float forwardNow = 0;              // smoothed values actually sent
    float turnNow    = 0;

    static float rampToward(float current, float target, float up, float down) {
        bool speedingUp = (fabsf(target) > fabsf(current)) && (target * current >= 0);
        float step = speedingUp ? up : down;
        if (target > current) return fminf(current + step, target);
        if (target < current) return fmaxf(current - step, target);
        return current;
    }
    static float curve(float x)     { return x * fabsf(x); }                  // gentle drive
    static float turnCurve(float x) { return x * (0.5f + 0.5f * fabsf(x)); }  // prompt turns
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // ---------- Arm (reversed: lower angle = higher) ----------
    float armTarget = 90;
    const float ARM_STEP = 0.6f;       // 60 deg/sec
    const float ARM_MIN  = 0;          // fully up   (pull back if it hits the frame)
    const float ARM_MAX  = 180;        // fully down

    // ---------- Gripper (NRL kit: 0 = closed, 180 = open) ----------
    float gripTarget = 90;
    const float GRIP_STEP = 1.5f;      // 150 deg/sec
    const float GRIP_MIN  = 0;         // most closed - raise a little once the claws meet
    const float GRIP_MAX  = 120;       // most open

    bool _dropOpen = false;

    // ---------- Tilt / docking state ----------
    float _tiltX = 0, _tiltY = 0;      // filtered tilt about each IMU axis (deg)
    float _baseX = 0, _baseY = 0;      // flat-floor reading, measured at START
    int   _baseCount = 0;
    bool  _docking   = false;
    bool  _dockLocked = false;         // uphill direction found
    bool  _dockUseX  = true;           // which IMU axis is our pitch
    float _dockSign  = 1.0f;           // so that "uphill" reads positive
    float _pitch     = 0, _pitchPrev = 0, _pitchRate = 0;
    float _dockHeading = 0;
    uint32_t _lastMs = 0;

    // ---------- OLED ----------
    HexaOLED _oled{ hexaOledConfig() };
    bool _oledOk = false;
    int  _oledTick = 0;

public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        gripper.begin();
        dropper.begin();                // starts CLOSED
        imu.begin();
        _oledOk = _oled.begin();
        armTarget = 90; gripTarget = 90; _dropOpen = false;
        forwardNow = 0; turnNow = 0; mode = &NORMAL;
        _tiltX = _tiltY = _baseX = _baseY = 0; _baseCount = 0;
        _docking = _dockLocked = false;
        _lastMs = millis();
    }

    void loop() override {
        // ---------- Tilt from the IMU ----------
        uint32_t now = millis();
        float dt = (now - _lastMs) / 1000.0f;
        _lastMs = now;
        imu.tick(now);
        imu.read();
        float rawX = atan2f(imu.getAccelX(), imu.getAccelZ()) * 57.2958f;
        float rawY = atan2f(imu.getAccelY(), imu.getAccelZ()) * 57.2958f;
        _tiltX += 0.15f * (rawX - _tiltX);              // smooth out bumps
        _tiltY += 0.15f * (rawY - _tiltY);
        // First 0.5 s after START (robot still on the flat floor): learn "level".
        if (_baseCount < 50) {
            _baseX += rawX / 50.0f;
            _baseY += rawY / 50.0f;
            _baseCount++;
        }
        float tX = _tiltX - _baseX;
        float tY = _tiltY - _baseY;

        // ---------- Drive mode: right toggle switch ----------
        mode = gamepad1.pressed(SW_ECO) ? &ECO : &NORMAL;

        // ---------- Driving ----------
        float forwardWanted = curve(gamepad1.leftY());
        float turnWanted    = turnCurve(gamepad1.rightX());
        forwardNow = rampToward(forwardNow, forwardWanted, mode->fwdAccel,  mode->fwdDecel);
        turnNow    = rampToward(turnNow,    turnWanted,    mode->turnAccel, mode->turnDecel);

        // ---------- Auto-dock (HOLD D-pad left) ----------
        if (gamepad1.justPressed(BTN_DOCK)) {
            _dockHeading = NRLComms::getHeading();
            _dockLocked = false;
            _pitchRate = 0;
        }
        _docking = gamepad1.pressed(BTN_DOCK);

        if (_docking) {
            float fwd = 0;

            // 1. Find which way is uphill: we drove FORWARD onto the slope,
            //    so whatever tilt we feel now = "nose up" = positive.
            if (!_dockLocked) {
                if (fabsf(tX) >= DOCK_LOCK_DEG || fabsf(tY) >= DOCK_LOCK_DEG) {
                    _dockUseX = fabsf(tX) >= fabsf(tY);
                    float v = _dockUseX ? tX : tY;
                    _dockSign = (v >= 0) ? 1.0f : -1.0f;
                    _dockLocked = true;
                    _pitchPrev = fabsf(v);
                } else {
                    fwd = DOCK_SEEK;            // not on the slope yet: creep forward
                }
            }

            // 2. Balance: tilted up -> drive forward, tipped past -> back up.
            if (_dockLocked) {
                _pitch = _dockSign * (_dockUseX ? tX : tY);
                if (dt > 0.0f && dt < 0.1f)
                    _pitchRate += 0.3f * (((_pitch - _pitchPrev) / dt) - _pitchRate);
                _pitchPrev = _pitch;

                if (fabsf(_pitch) > BAL_LEVEL) {
                    fwd = BAL_KP * _pitch + BAL_KD * _pitchRate;
                    if (fabsf(fwd) < BAL_MIN) fwd = (_pitch > 0) ? BAL_MIN : -BAL_MIN;
                    fwd = clampf(fwd, -BAL_MAX, BAL_MAX);
                }                                // else level: hold still
            }

            // 3. Stay square: hold the heading we had when the button was pressed.
            float hdgErr = NRLComms::getHeading() - _dockHeading;
            while (hdgErr > 180)  hdgErr -= 360;
            while (hdgErr < -180) hdgErr += 360;
            float turn = clampf(-BAL_HDG_KP * hdgErr, -0.3f, 0.3f);

            drive.setScale(1.0f);
            drive.drive(fwd, turn);
            forwardNow = 0;                      // no lurch when the button is released
            turnNow    = 0;
        } else {
            _dockLocked = false;
            _pitch = 0;
            float turn = turnNow - DRIVE_TRIM * forwardNow;
            drive.setScale(mode->topSpeed);
            drive.drive(forwardNow, turn);
        }
        bool level = _docking && _dockLocked && fabsf(_pitch) <= BAL_LEVEL;

        // ---------- Arm ----------
        if (gamepad1.pressed(BTN_ARM_UP))   armTarget -= ARM_STEP;
        if (gamepad1.pressed(BTN_ARM_DOWN)) armTarget += ARM_STEP;
        armTarget = clampf(armTarget, ARM_MIN, ARM_MAX);
        arm.setPosition(armTarget);

        // ---------- Gripper ----------
        if (gamepad1.pressed(BTN_GRIP_OPEN))  gripTarget += GRIP_STEP;
        if (gamepad1.pressed(BTN_GRIP_CLOSE)) gripTarget -= GRIP_STEP;
        gripTarget = clampf(gripTarget, GRIP_MIN, GRIP_MAX);
        gripper.setPosition(gripTarget);

        // ---------- Payload dropper ----------
        if (gamepad1.justPressed(BTN_DROPPER)) {
            _dropOpen = !_dropOpen;
            dropper.setPosition(_dropOpen ? DROP_OPEN : DROP_CLOSED);
        }

        // ---------- Heading reset ----------
        if (gamepad1.justPressed(BTN_A)) NRLComms::zeroHeading();

        // ---------- Status LED ----------
        if (power.isBatteryLow())  userLed.setBlink(255, 0, 0, 250);   // red blink: low battery
        else if (level)            userLed.setSolid(255, 255, 255);    // white: docked and level
        else if (_docking)         userLed.setBlink(0, 200, 200, 200); // cyan blink: docking
        else if (mode == &ECO)     userLed.setSolid(0, 80, 0);         // green: ECO
        else                       userLed.setSolid(0, 0, 80);         // blue: NORMAL

        // ---------- On-robot OLED ----------
        if (_oledOk && ++_oledTick >= 10) {
            _oledTick = 0;
            _oled.clearDisplay();
            _oled.setTextSize(1);
            _oled.setTextColor(HexaOLED::WHITE);
            _oled.setCursor(0, 0);
            _oled.print(_docking ? "DOCKING" : mode->name);
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
            _oled.display();
        }

        // ---------- Telemetry (key <= 11 chars, value <= 15) ----------
        telemetry.addData("mode", mode->name);
        telemetry.addData("docking", !_docking ? "OFF"
                                   : !_dockLocked ? "ON - FINDING"
                                   : level ? "ON - LEVEL" : "ON");
        telemetry.addData("pitch", _docking ? _pitch : (fabsf(tX) >= fabsf(tY) ? tX : tY));
        telemetry.addData("heading", NRLComms::isHeadingReady() ? NRLComms::getHeading() : -1.0f);
        telemetry.addData("hdg status", NRLComms::isHeadingReady() ? "READY" : "CALIBRATING");
        telemetry.addData("battery V", power.getBatteryVoltage());
        telemetry.addData("servo A", power.getServoCurrent());
        telemetry.addData("arm", armTarget);
        telemetry.addData("gripper", gripTarget);
        telemetry.addData("dropper", _dropOpen ? "OPEN" : "CLOSED");
    }

    void stop() override {
        drive.stop();
        userLed.setOff();
        arm.detach();
        gripper.detach();
        dropper.detach();
    }
};

REGISTER_OPMODE(Oakbots, "Oakbots", TELEOP);
