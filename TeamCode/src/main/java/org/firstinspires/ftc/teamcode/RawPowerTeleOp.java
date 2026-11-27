package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Same controls as Final TeleOp, but the flywheel runs at a plain fixed power.
 * No PIDF, no feedforward, no bang-bang, no voltage scaling: what you set is what the
 * motor gets. Use it to see how strong the outtake is at full power.
 *
 * Driver 1 (gamepad1):
 *   Left stick      - drive / strafe      Right stick X - turn
 *   Right bumper    - hold for slow mode
 *   Left trigger    - hold: intake
 *   Left bumper     - hold: both transfer motors
 *   Right trigger   - hold: flywheel at the set power
 *   Y               - toggle flywheel on/off without holding RT
 *   A / B           - ball block OPEN / CLOSED
 *   Dpad Up / Down  - flywheel power +/- POWER_STEP (0 to 1)
 *   Dpad Left       - reset power to START_POWER
 *
 * The encoder is only read for telemetry (flywheel RPM), never used for control.
 * Nothing moves during INIT (G403).
 */
@Config
@TeleOp(name = "Raw Power TeleOp", group = "Test")
public class RawPowerTeleOp extends LinearOpMode {

    public static double START_POWER = 1.0;  // flywheel power at START
    public static double POWER_STEP  = 0.05; // dpad up/down change per press

    private Drivetrain drivetrain;
    private Intake intake;
    private DcMotorEx flywheel;
    private Servo ballBlock;
    private VoltageSensor battery;

    private double flywheelPower = START_POWER;
    private boolean flywheelToggle = false;
    private boolean blockOpen = false;
    private boolean prevY = false, prevDpadUp = false, prevDpadDown = false;

    private double peakRpm = 0; // highest flywheel RPM seen while spinning

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);

        // Same motor setup as Shooter, so direction and RPM readings match.
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ballBlock = hardwareMap.get(Servo.class, Constants.BALL_BLOCK_NAME);
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Raw power: no PIDF. Nothing moves until START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            adjustPower();
            mechanisms();
            sendTelemetry();
        }

        drivetrain.stop();
        intake.stopFeed();
        flywheel.setPower(0);
    }

    private void drive() {
        double forward = shape(-gamepad1.left_stick_y);
        double strafe  = shape(gamepad1.left_stick_x);
        double turn    = shape(gamepad1.right_stick_x) * Constants.TURN_CAP;
        if (Constants.INVERT_FORWARD) forward = -forward;
        if (Constants.INVERT_STRAFE)  strafe  = -strafe;
        if (Constants.INVERT_TURN)    turn    = -turn;

        if (gamepad1.right_bumper) {
            forward *= Constants.SLOW_MODE;
            strafe  *= Constants.SLOW_MODE;
            turn    *= Constants.SLOW_MODE;
        }
        drivetrain.drive(forward, strafe, turn);
    }

    private void adjustPower() {
        if (gamepad1.dpad_up && !prevDpadUp) {
            flywheelPower = Math.min(flywheelPower + POWER_STEP, 1.0);
        }
        if (gamepad1.dpad_down && !prevDpadDown) {
            flywheelPower = Math.max(flywheelPower - POWER_STEP, 0.0);
        }
        if (gamepad1.dpad_left) {
            flywheelPower = START_POWER;
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    private void mechanisms() {
        if (gamepad1.y && !prevY) {
            flywheelToggle = !flywheelToggle;
        }
        prevY = gamepad1.y;

        boolean flywheelOn = gamepad1.right_trigger > Constants.TRIGGER_THRESHOLD || flywheelToggle;
        flywheel.setPower(flywheelOn ? flywheelPower : 0);
        if (!flywheelOn) peakRpm = 0;

        intake.setIntake(gamepad1.left_trigger > Constants.TRIGGER_THRESHOLD);
        intake.setTransfer(gamepad1.left_bumper);

        if (gamepad1.a) {
            blockOpen = true;
        } else if (gamepad1.b) {
            blockOpen = false;
        }
        ballBlock.setPosition(blockOpen ? Constants.BALL_BLOCK_OPEN : Constants.BALL_BLOCK_CLOSED);
    }

    /** Flywheel RPM from the encoder (same math as FlywheelPIDF.getRPM). Telemetry only. */
    private double flywheelRpm() {
        return -flywheel.getVelocity() / FlywheelPIDF.TICKS_PER_REV * 60.0;
    }

    private static double shape(double value) {
        if (Math.abs(value) < Constants.DEADZONE) return 0;
        return Math.copySign(value * value, value);
    }

    private void sendTelemetry() {
        double rpm = flywheelRpm();
        if (flywheel.getPower() > 0) peakRpm = Math.max(peakRpm, rpm);

        telemetry.addData("Flywheel power (dpad)", flywheelPower);
        telemetry.addData("Flywheel on (RT / Y)", flywheel.getPower() > 0 ? "ON" : "off");
        // Plain numbers so FTC Dashboard can graph them
        telemetry.addData("Flywheel RPM", rpm);
        telemetry.addData("Peak RPM", peakRpm);
        telemetry.addData("Flywheel current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.addData("Intake (LT)", gamepad1.left_trigger > Constants.TRIGGER_THRESHOLD ? "ON" : "off");
        telemetry.addData("Transfer (LB)", gamepad1.left_bumper ? "ON" : "off");
        telemetry.addData("Ball block", blockOpen ? "OPEN" : "CLOSED");
        telemetry.addLine("RT/Y=flywheel  Dpad U/D=power  Dpad L=reset  LT=intake  LB=transfer  A/B=block  RB=slow");
        telemetry.update();
    }
}
