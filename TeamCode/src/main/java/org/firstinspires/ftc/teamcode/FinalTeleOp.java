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
 *   Left trigger    - INTAKE: intake + transfer on, ball block closed (balls held)
 *   Right trigger   - SHOOT: flywheel to TELEOP_SHOT_RPM, feeds only once the
 *                     flywheel is ready. Both triggers held = shoot.
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

    private enum IntakeState { IDLE, INTAKING, SHOOTING_WAITING, SHOOTING_FEEDING }

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    // Driver 1 state
    private boolean slowMode = false;
    private IntakeState intakeState = IntakeState.IDLE;

    // Previous-loop values, for edge detection (react to the change, not every loop)
    private boolean prevShooterReady = false;

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
        intakeAndShoot();
    }

    private void driveFromSticks() {
        slowMode = gamepad1.right_bumper;

        // Stick up is negative on the gamepad, so flip Y.
        double forward = shape(-gamepad1.left_stick_y);
        double strafe  = shape(gamepad1.left_stick_x);
        double turn    = shape(gamepad1.right_stick_x) * Constants.TURN_CAP;

        if (slowMode) {
            forward *= Constants.SLOW_MODE;
            strafe  *= Constants.SLOW_MODE;
            turn    *= Constants.SLOW_MODE;
        }

        drivetrain.drive(forward, strafe, turn);
    }

    private void intakeAndShoot() {
        boolean shootHeld  = gamepad1.right_trigger > Constants.TRIGGER_THRESHOLD;
        boolean intakeHeld = gamepad1.left_trigger  > Constants.TRIGGER_THRESHOLD;

        if (shootHeld) {
            // Shooting wins if both triggers are held.
            shooter.setTargetRpm(Constants.TELEOP_SHOT_RPM);
            if (shooter.isReady()) {
                shooter.openBallBlock();
                intake.feed();
                intakeState = IntakeState.SHOOTING_FEEDING;
            } else {
                // Wait at speed-up with balls held back; feeding starts by itself once ready.
                shooter.closeBallBlock();
                intake.stopFeed();
                intakeState = IntakeState.SHOOTING_WAITING;
            }
        } else if (intakeHeld) {
            shooter.stop();
            shooter.closeBallBlock();
            intake.intake();
            intakeState = IntakeState.INTAKING;
        } else {
            shooter.stop();
            shooter.closeBallBlock();
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
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Current RPM", shooter.getCurrentRpm());
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Ball block", intakeState == IntakeState.SHOOTING_FEEDING
                ? "OPEN (" + Constants.BALL_BLOCK_OPEN + ")"
                : "CLOSED (" + Constants.BALL_BLOCK_CLOSED + ")");
        telemetry.addData("Wheels FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f",
                drivetrain.getFrontLeftPower(), drivetrain.getFrontRightPower(),
                drivetrain.getBackLeftPower(), drivetrain.getBackRightPower());
        telemetry.addLine("LS=drive/strafe  RS X=turn  RB=slow  LT=intake  RT=shoot");
        telemetry.update();
    }
}
