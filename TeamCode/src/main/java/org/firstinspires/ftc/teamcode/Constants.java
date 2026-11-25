package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // --- Hardware config names (final, not shown in Dashboard) ---
    public static final String FLYWHEEL_NAME   = "shooter";
    public static final String TRANSFER_NAME   = "transferMotor";
    public static final String TRANSFER2_NAME  = "transferMotor2";
    public static final String INTAKE_NAME     = "intakeMotor";
    public static final String BALL_BLOCK_NAME = "ballBlock";

    // TODO check: drive motor and Pinpoint names are placeholders. They must match
    // the Driver Station config exactly (case-sensitive).
    public static final String FRONT_LEFT_NAME  = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME   = "backLeft";
    public static final String BACK_RIGHT_NAME  = "backRight";
    public static final String PINPOINT_NAME    = "pinpoint";

    // --- Drivetrain directions ---
    // TODO check on blocks: pushing forward should spin every wheel forward.
    // Flip the one that's wrong. Usually one whole side is reversed.
    public static boolean FRONT_LEFT_REVERSED  = true;
    public static boolean BACK_LEFT_REVERSED   = true;
    public static boolean FRONT_RIGHT_REVERSED = false;
    public static boolean BACK_RIGHT_REVERSED  = false;

    // --- Driver 1 controls ---
    public static double DEADZONE          = 0.1;  // stick values smaller than this count as 0
    public static double TURN_CAP          = 0.8;  // max turn power (after squaring)
    public static double SLOW_MODE         = 0.4;  // drive scale while right bumper is held
    public static double TRIGGER_THRESHOLD = 0.5;  // trigger counts as pressed above this

    // --- Ball block servo ---
    // Used by FinalTeleOp, BallBlockTest (A/B buttons) and BiobuzzAuto.
    public static double BALL_BLOCK_OPEN   = 0.47;  // balls can pass into the flywheel
    public static double BALL_BLOCK_CLOSED = 0.70;  // balls are held back

    // --- Flywheel ---
    public static double RPM_STEP  = 100;   // RPM change per dpad press
    public static double START_RPM = 1000;  // initial target RPM

    // --- Motor powers ---
    public static double INTAKE_POWER    = -1.0;
    public static double TRANSFER_POWER  = 1.0;
    public static double TRANSFER2_POWER = 1.0;  // flip sign if it spins the wrong way

    // The sign of INTAKE_POWER / TRANSFER_POWER / TRANSFER2_POWER sets which way
    // each motor spins. Feeding uses the same signs at FEED_POWER strength.
    public static double FEED_POWER = 0.7;       // intake + transfer power while feeding a shot

    // --- Shooter (velocity control) ---
    public static double NOMINAL_VOLTAGE = 12.0; // battery voltage the flywheel gains were tuned at
    public static double RPM_TOLERANCE   = 75;   // TODO measure: |target - actual| counted as "at speed"
    public static double READY_HOLD_SEC  = 0.12; // must stay at speed this long before feeding

    // --- Auto shooting ---
    public static double START_SHOT_RPM   = 2000; // TODO measure: RPM that scores from the start pose
    public static double SHOT_DIP_RPM     = 200;  // TODO measure: RPM drop below target that means a ball went through
    public static int    PRELOAD_COUNT    = 4;    // balls shot in auto (G407: never more than 4)
    public static double SPIN_UP_TIMEOUT  = 2.5;  // s, give up shooting and go park if never at speed
    public static double FEED_TIMEOUT     = 1.0;  // s, per ball: stop feeding if no dip is seen
    public static double RECOVER_TIMEOUT  = 0.8;  // s, feed the next ball even if not fully back to speed
    public static double SHOOT_TIME_CAP   = 5.0;  // s, total shooting time before we leave to park
    public static double AUTO_STOP_TIME   = 29.5; // s, everything stops by here (AUTO is 30 s)

    // --- TeleOp shooting ---
    public static double TELEOP_SHOT_RPM = 2000;  // TODO tune: fixed RPM while left trigger is held
    public static int    READY_RUMBLE_MS = 200;   // rumble length when the shooter becomes ready
}