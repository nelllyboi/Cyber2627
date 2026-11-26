package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Final TeleOp for Cyber2627.
 *
 * Driver 1 (gamepad1) drives, intakes and shoots:
 *   Left stick Y    - forward / back
 *   Left stick X    - strafe
 *   Right stick X   - turn (capped at TURN_CAP)
 *   Right bumper    - hold for slow mode (SLOW_MODE)
 *   Left trigger    - INTAKE: intake + transfer on
 *   Right trigger   - SHOOT: flywheel to TELEOP_SHOT_RPM; intake/transfer feed
 *                     only once the flywheel is ready
 *   Both triggers   - SHOOT + TRANSFER: flywheel and intake/transfer all on
 *   A               - ball block OPEN
 *   B               - ball block CLOSED
 *                     The ball block is ONLY moved by A/B (starts CLOSED at START).
 *   Dpad Up / Down  - shot RPM +/- RPM_STEP (between 0 and TELEOP_MAX_RPM)
 *   Dpad Left       - reset shot RPM to TELEOP_SHOT_RPM
 *   Y               - toggle flywheel spinning at shot RPM without holding RT
 *                     (no feeding; for PIDF tuning and pre-spinning)
 *
 * PIDF tuning (same as the Nov 11 TeleOp): open FTC Dashboard
 * (http://192.168.43.1:8080/dash), graph Target RPM / Current RPM / Error RPM,
 * and edit kP/kS/kV/kA under FlywheelPIDF in the Configuration tab (save each edit).
 *
 * Driving is robot-centric: forward is always the way the robot faces.
 *   Rumble          - short buzz when the shooter becomes ready
 *
 * Driver 2 (gamepad2): not yet. Add controls in handleDriver2().
 *
 * Every stick input is: deadzone -> square (keeping the sign) -> scale.
 * Squaring gives fine control near the center and full power at the edge.
 *
 * Nothing moves during INIT (G403). All numbers live in Constants.java.
 */
@TeleOp(name = "Final TeleOp", group = "Competition")
public class FinalTeleOp extends LinearOpMode {

    private enum IntakeState { IDLE, INTAKING, SHOOTING_WAITING, SHOOTING_FEEDING, SHOOT_AND_TRANSFER }

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    // Driver 1 state
    private boolean slowMode = false;
    private IntakeState intakeState = IntakeState.IDLE;
    private boolean blockOpen = false; // ball block: A = open, B = closed
    private double shotRpm = Constants.TELEOP_SHOT_RPM; // changed with dpad up/down/left

    // Previous-loop values, for edge detection (react to the change, not every loop)
    private boolean prevShooterReady = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevY = false;
    private boolean flywheelToggle = false; // Y: keep the flywheel spinning without RT

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        telemetry.addLine("Ready. Nothing moves until START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriver1();
            handleDriver2();

            shooter.update();
            sendTelemetry();
        }

        drivetrain.stop();
        intake.stopFeed();
        shooter.stop();
        shooter.update();
    }

    // ---------------------------------------------------------------------
    // Driver 1
    // ---------------------------------------------------------------------

    private void handleDriver1() {
        driveFromSticks();
        adjustShotRpm();
        intakeAndShoot();
    }

    /** Dpad up/down: shot RPM +/- RPM_STEP (once per press). Dpad left: reset. */
    private void adjustShotRpm() {
        if (gamepad1.dpad_up && !prevDpadUp) {
            shotRpm = Math.min(shotRpm + Constants.RPM_STEP, Constants.TELEOP_MAX_RPM);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            shotRpm = Math.max(shotRpm - Constants.RPM_STEP, 0);
        }
        if (gamepad1.dpad_left) {
            shotRpm = Constants.TELEOP_SHOT_RPM;
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    private void driveFromSticks() {
        slowMode = gamepad1.right_bumper;

        // Stick up is negative on the gamepad, so flip Y.
        double forward = shape(-gamepad1.left_stick_y);
        double strafe  = shape(gamepad1.left_stick_x);
        double turn    = shape(gamepad1.right_stick_x) * Constants.TURN_CAP;
        if (Constants.INVERT_FORWARD) forward = -forward;
        if (Constants.INVERT_STRAFE)  strafe  = -strafe;
        if (Constants.INVERT_TURN)    turn    = -turn;

        if (slowMode) {
            forward *= Constants.SLOW_MODE;
            strafe  *= Constants.SLOW_MODE;
            turn    *= Constants.SLOW_MODE;
        }

        drivetrain.drive(forward, strafe, turn);
    }

    private void intakeAndShoot() {
        // Y: toggle the flywheel on/off (spins at shot RPM, never feeds by itself)
        if (gamepad1.y && !prevY) {
            flywheelToggle = !flywheelToggle;
        }
        prevY = gamepad1.y;

        boolean shootHeld  = gamepad1.right_trigger > Constants.TRIGGER_THRESHOLD;
        boolean intakeHeld = gamepad1.left_trigger  > Constants.TRIGGER_THRESHOLD;

        // Ball block: ONLY A and B move it. The triggers never touch it.
        if (gamepad1.a) {
            blockOpen = true;
        } else if (gamepad1.b) {
            blockOpen = false;
        }
        if (blockOpen) {
            shooter.openBallBlock();
        } else {
            shooter.closeBallBlock();
        }

        if (shootHeld && intakeHeld) {
            // Flywheel and intake/transfer both on at full power.
            shooter.setTargetRpm(shotRpm);
            intake.intake();
            intakeState = IntakeState.SHOOT_AND_TRANSFER;
        } else if (shootHeld) {
            // Flywheel on; intake/transfer feed only once the flywheel is ready.
            shooter.setTargetRpm(shotRpm);
            if (shooter.isReady()) {
                intake.feed();
                intakeState = IntakeState.SHOOTING_FEEDING;
            } else {
                intake.stopFeed();
                intakeState = IntakeState.SHOOTING_WAITING;
            }
        } else if (intakeHeld) {
            spinOrStopFlywheel();
            intake.intake();
            intakeState = IntakeState.INTAKING;
        } else {
            spinOrStopFlywheel();
            intake.stopFeed();
            intakeState = IntakeState.IDLE;
        }

        // Rumble once on the change from not-ready to ready.
        boolean ready = shooter.isReady();
        if (ready && !prevShooterReady) {
            gamepad1.rumble(Constants.READY_RUMBLE_MS);
        }
        prevShooterReady = ready;
    }

    /** With no RT held: keep spinning if Y toggled the flywheel on, otherwise stop it. */
    private void spinOrStopFlywheel() {
        if (flywheelToggle) {
            shooter.setTargetRpm(shotRpm);
        } else {
            shooter.stop();
        }
    }

    // ---------------------------------------------------------------------
    // Driver 2 (gamepad2) - TODO
    // Planned: RPM offsets, calibration, failsafes. Read gamepad2 here and use
    // the same edge-detection pattern as prevShooterReady above.
    // ---------------------------------------------------------------------

    private void handleDriver2() {
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    /** Deadzone, then square while keeping the sign. */
    private static double shape(double value) {
        if (Math.abs(value) < Constants.DEADZONE) return 0;
        return Math.copySign(value * value, value);
    }

    private void sendTelemetry() {
        telemetry.addData("Drive mode", "ROBOT-CENTRIC");
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake state", intakeState);
        telemetry.addData("Shot RPM (dpad)", shotRpm);
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Current RPM", shooter.getCurrentRpm());
        telemetry.addData("Flywheel on (Y)", flywheelToggle);
        // Plain numbers (no format strings) so FTC Dashboard can graph them
        telemetry.addData("Error RPM", shooter.getTargetRpm() > 0
                ? shooter.getTargetRpm() - shooter.getCurrentRpm() : 0);
        telemetry.addData("Flywheel current (A)", shooter.getFlywheelCurrent());
        telemetry.addData("Encoder (ticks/s)", shooter.getEncoderTicksPerSec());
        telemetry.addData("Encoder position (ticks)", shooter.getEncoderPosition());
        telemetry.addData("Flywheel power", shooter.getPower());
        telemetry.addData("Battery (V)", shooter.getBatteryVoltage());

        // Flywheel tuning constants (FlywheelPIDF + Constants)
        telemetry.addData("kP", FlywheelPIDF.kP);
        telemetry.addData("kI", FlywheelPIDF.kI);
        telemetry.addData("kD", FlywheelPIDF.kD);
        telemetry.addData("kS", FlywheelPIDF.kS);
        telemetry.addData("kV", FlywheelPIDF.kV);
        telemetry.addData("kA", FlywheelPIDF.kA);
        telemetry.addData("Ticks per rev", FlywheelPIDF.TICKS_PER_REV);
        telemetry.addData("Nominal voltage", Constants.NOMINAL_VOLTAGE);
        telemetry.addData("RPM tolerance", Constants.RPM_TOLERANCE);
        telemetry.addData("Ready hold (s)", Constants.READY_HOLD_SEC);
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Ball block", blockOpen
                ? "OPEN (" + Constants.BALL_BLOCK_OPEN + ")"
                : "CLOSED (" + Constants.BALL_BLOCK_CLOSED + ")");
        telemetry.addData("Wheels FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f",
                drivetrain.getFrontLeftPower(), drivetrain.getFrontRightPower(),
                drivetrain.getBackLeftPower(), drivetrain.getBackRightPower());
        telemetry.addLine("LS=drive/strafe  RS X=turn  RB=slow  LT=intake  RT=shoot  LT+RT=shoot+transfer  A=block open  B=block closed  Dpad U/D=RPM  Dpad L=reset RPM  Y=flywheel on/off");
        telemetry.update();

    }
}
