package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Finds the servo positions for the flower intake bar (2 servos).
 *
 * Only the servo you pick moves. The other one is switched off (goes limp), so the
 * two servos never fight each other on the bar while you tune.
 *
 * Gamepad 1:
 *   X               - pick flower1 servo  B - pick flower2 servo
 *   Right bumper    - pick BOTH. Only works once all 4 positions are saved. Both
 *                     servos go to their own saved UP together; then only LB moves
 *                     them (Dpad and Y/A are locked out so they can never fight).
 *   Dpad Up / Down  - position +/- FINE_STEP (0.01)
 *   Dpad Right/Left - position +/- COARSE_STEP (0.05)
 *   Y               - save current position as UP   (bar raised)
 *   A               - save current position as DOWN (bar lowered into the flower)
 *   Left bumper     - jump the picked servo(s) to its saved UP, press again for DOWN
 *
 * Nothing moves during INIT or right after START. A servo only gets power the
 * first time you press a dpad button with it picked, starting at START_POS.
 * First time: keep hands clear and use small steps, a servo pushing the bar into
 * the frame can strip its gears. If it buzzes and won't move, it's pushing against
 * something: back off.
 */
@Config
@TeleOp(name = "Flower Servo Tuner", group = "Test")
public class FlowerServoTuner extends LinearOpMode {

    public static double START_POS   = 0.5;  // where a servo starts the first time it's moved
    public static double FINE_STEP   = 0.01;
    public static double COARSE_STEP = 0.05;

    private ServoImplEx left, right;
    private double leftPos = START_POS, rightPos = START_POS;
    private boolean leftOn = false, rightOn = false; // has this servo been powered yet
    private double leftUp = -1, leftDown = -1, rightUp = -1, rightDown = -1; // -1 = not saved
    private String picked = "FLOWER1";
    private boolean goUpNext = true;
    private String message = "Unplug the servo you're NOT tuning, to be extra safe.";

    private boolean prevUp, prevDown, prevRight, prevLeft, prevY, prevA, prevLB;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left  = hardwareMap.get(ServoImplEx.class, Constants.FLOWER1_NAME);
        right = hardwareMap.get(ServoImplEx.class, Constants.FLOWER2_NAME);
        left.setPwmDisable();  // limp until used
        right.setPwmDisable();

        telemetry.addLine("Flower servo tuner. Nothing moves until you press a dpad button.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pickServo();
            double step = 0;
            if (gamepad1.dpad_up && !prevUp)       step = FINE_STEP;
            if (gamepad1.dpad_down && !prevDown)   step = -FINE_STEP;
            if (gamepad1.dpad_right && !prevRight) step = COARSE_STEP;
            if (gamepad1.dpad_left && !prevLeft)   step = -COARSE_STEP;
            if (step != 0) nudge(step);

            if (gamepad1.y && !prevY) save(true);
            if (gamepad1.a && !prevA) save(false);
            if (gamepad1.left_bumper && !prevLB) jumpToSaved();

            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            prevRight = gamepad1.dpad_right; prevLeft = gamepad1.dpad_left;
            prevY = gamepad1.y; prevA = gamepad1.a; prevLB = gamepad1.left_bumper;

            applyPower();
            sendTelemetry();
        }
        left.setPwmDisable();
        right.setPwmDisable();
    }

    private void pickServo() {
        if (gamepad1.x) picked = "FLOWER1";
        if (gamepad1.b) picked = "FLOWER2";
        if (gamepad1.right_bumper && !picked.equals("BOTH")) {
            // SAFETY: both servos only get power together at a matching saved pair,
            // never at whatever leftover positions they had, or they could fight.
            if (leftUp >= 0 && leftDown >= 0 && rightUp >= 0 && rightDown >= 0) {
                picked = "BOTH";
                leftPos = leftUp;
                rightPos = rightUp;
                leftOn = rightOn = true;
                goUpNext = false; // next LB goes DOWN
                message = "BOTH: moved to UP. LB toggles DOWN/UP. Dpad and Y/A are off.";
            } else {
                message = "Can't pick BOTH yet: save UP and DOWN for each servo first.";
            }
        }
    }

    private boolean usesLeft()  { return !picked.equals("FLOWER2"); }
    private boolean usesRight() { return !picked.equals("FLOWER1"); }

    private void nudge(double step) {
        if (picked.equals("BOTH")) return; // SAFETY: same step could turn them opposite ways
        if (usesLeft())  { leftPos  = clip(leftPos + step);  leftOn = true; }
        if (usesRight()) { rightPos = clip(rightPos + step); rightOn = true; }
    }

    private void save(boolean up) {
        if (picked.equals("BOTH")) return;
        if (usesLeft())  { if (up) leftUp = leftPos;   else leftDown = leftPos; }
        if (usesRight()) { if (up) rightUp = rightPos; else rightDown = rightPos; }
    }

    private void jumpToSaved() {
        if (usesLeft()) {
            double target = goUpNext ? leftUp : leftDown;
            if (target >= 0) { leftPos = target; leftOn = true; }
        }
        if (usesRight()) {
            double target = goUpNext ? rightUp : rightDown;
            if (target >= 0) { rightPos = target; rightOn = true; }
        }
        goUpNext = !goUpNext;
    }

    /** Only the picked servo(s) get power; the other goes limp so they can't fight. */
    private void applyPower() {
        if (usesLeft() && leftOn) { left.setPwmEnable(); left.setPosition(leftPos); }
        else left.setPwmDisable();
        if (usesRight() && rightOn) { right.setPwmEnable(); right.setPosition(rightPos); }
        else right.setPwmDisable();
    }

    private static double clip(double v) { return Math.max(0, Math.min(1, v)); }

    private static String fmt(double v) { return v < 0 ? "not saved" : String.format("%.3f", v); }

    private void sendTelemetry() {
        telemetry.addData("Picked (X=flower1 B=flower2 RB=both)", picked);
        telemetry.addLine(message);
        telemetry.addData("flower1 position", "%.3f %s", leftPos, usesLeft() && leftOn ? "(powered)" : "(limp)");
        telemetry.addData("flower2 position", "%.3f %s", rightPos, usesRight() && rightOn ? "(powered)" : "(limp)");
        telemetry.addLine();
        telemetry.addData("flower1 UP (Y)", fmt(leftUp));
        telemetry.addData("flower1 DOWN (A)", fmt(leftDown));
        telemetry.addData("flower2 UP (Y)", fmt(rightUp));
        telemetry.addData("flower2 DOWN (A)", fmt(rightDown));
        telemetry.addData("LB next jumps to", goUpNext ? "UP" : "DOWN");
        telemetry.addLine();
        telemetry.addLine("Dpad U/D = 0.01, Dpad R/L = 0.05. Write the 4 saved numbers down.");
        telemetry.update();
    }
}
