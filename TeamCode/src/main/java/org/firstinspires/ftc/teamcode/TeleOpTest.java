package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Flywheel tuning TeleOp for Cyber2627.
 *
 * The flywheel control math lives in {@link FlywheelPIDF}. Ball block positions
 * come from {@link BallBlockTest} (BLOCK_MIN / BLOCK_MAX).
 *
 * Live tuning: open FTC Dashboard (http://192.168.43.1:8080/dash), add
 * Target RPM and Current RPM to the Graph, and edit kP / kV / kS / etc. under
 * FlywheelPIDF in the Configuration tab. Hit save on each field after editing.
 * Copy final values into the code so they survive an app restart.
 *
 * Tuning order:
 *   kS = minimum power to make the flywheel start spinning  -> tune FIRST
 *   kV = power per RPM, gets you to the target RPM           -> tune SECOND
 *   kP = corrects the remaining error                        -> tune THIRD
 *   kA = extra power while the target RPM is changing        -> tune LAST
 *
 * Hardware config names (rename the constants below to match your Driver Station):
 *   "shooter"        - flywheel motor
 *   "transferMotor"  - transfer/feeder motor 1
 *   "transferMotor2" - transfer/feeder motor 2
 *   "intakeMotor"    - intake motor
 *   "ballBlock"      - ball block servo
 *
 * Controls (gamepad1):
 *   A            - toggle flywheel on/off
 *   B            - toggle both transfer motors on/off
 *   X            - toggle intake motor on/off
 *   Y            - toggle ball block (MIN <-> MAX)
 *   Dpad Up/Down - raise / lower target RPM by RPM_STEP
 */
@TeleOp(name = "TeleOp", group = "Test")
public class TeleOpTest extends LinearOpMode {

    // --- Hardware config names ---
    private static final String FLYWHEEL_NAME  = "shooter";
    private static final String TRANSFER_NAME  = "transferMotor";
    private static final String TRANSFER2_NAME = "transferMotor2";
    private static final String INTAKE_NAME    = "intakeMotor";
    private static final String BALL_BLOCK_NAME = BallBlockTest.BALL_BLOCK_NAME;

    // --- Tuning ---
    private static final double RPM_STEP        = 100;   // RPM change per dpad press
    private static final double START_RPM       = 1000;  // initial target RPM
    private static final double INTAKE_POWER    = -1.0;
    private static final double TRANSFER_POWER  = 1.0;
    private static final double TRANSFER2_POWER = 1.0;   // flip sign if it spins the wrong way

    private DcMotorEx flywheelMotor;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx transfer2;
    private Servo ballBlock;
    private FlywheelPIDF flywheelPID;

    private double targetRPM = START_RPM;

    private boolean flywheelOn = false;
    private boolean transferOn = false;
    private boolean intakeOn = false;
    private boolean blockOn = false;

    // Rising-edge state
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean dUPressed = false;
    private boolean dDPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry to both the Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer2 = hardwareMap.get(DcMotorEx.class, TRANSFER2_NAME);
        transfer2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ballBlock = hardwareMap.get(Servo.class, BALL_BLOCK_NAME);
        ballBlock.setPosition(BallBlockTest.BLOCK_MIN);

        // Single-motor flywheel. For a dual-motor flywheel, fetch a second
        // DcMotorEx and use: new FlywheelPIDF(hardwareMap, flywheelMotor, flywheelMotor2);
        flywheelPID = new FlywheelPIDF(hardwareMap, flywheelMotor);

        telemetry.addLine("Ready. A = flywheel, B = transfer, X = intake, Y = ball block, Dpad = RPM.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double currentRPM = flywheelPID.getRPM();
            double power = 0;

            // Transfer motors on/off (both together)
            if (transferOn) {
                transfer.setPower(TRANSFER_POWER);
                transfer2.setPower(TRANSFER2_POWER);
            } else {
                transfer.setPower(0);
                transfer2.setPower(0);
            }

            // Intake motor on/off
            if (intakeOn) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // Ball block servo (MAX = blocking, MIN = open)
            ballBlock.setPosition(blockOn ? BallBlockTest.BLOCK_MAX : BallBlockTest.BLOCK_MIN);

            // A: toggle flywheel
            if (gamepad1.a) {
                if (!aPressed) {
                    flywheelOn = !flywheelOn;
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            // B: toggle transfer (both motors)
            if (gamepad1.b) {
                if (!bPressed) {
                    transferOn = !transferOn;
                    bPressed = true;
                }
            } else {
                bPressed = false;
            }

            // X: toggle intake
            if (gamepad1.x) {
                if (!xPressed) {
                    intakeOn = !intakeOn;
                    xPressed = true;
                }
            } else {
                xPressed = false;
            }

            // Y: toggle ball block
            if (gamepad1.y) {
                if (!yPressed) {
                    blockOn = !blockOn;
                    yPressed = true;
                }
            } else {
                yPressed = false;
            }

            // Dpad up: raise target RPM
            if (gamepad1.dpad_up) {
                if (!dUPressed) {
                    targetRPM += RPM_STEP;
                    dUPressed = true;
                }
            } else {
                dUPressed = false;
            }

            // Dpad down: lower target RPM
            if (gamepad1.dpad_down) {
                if (!dDPressed) {
                    targetRPM -= RPM_STEP;
                    dDPressed = true;
                }
            } else {
                dDPressed = false;
            }

            // Drive the flywheel
            if (flywheelOn) {
                power = flywheelPID.controller(targetRPM, currentRPM);
                power = Math.max(0, Math.min(1, power));
                flywheelPID.setPower(power);
            } else {
                flywheelPID.setPower(0);
            }

            // Plain numbers (no format strings) so Dashboard can graph them
            telemetry.addData("Flywheel Active", flywheelOn ? "Yes" : "No");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Error RPM", flywheelOn ? (targetRPM - currentRPM) : 0);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Flywheel Current (A)", "%.2f", flywheelPID.getCurrent());
            telemetry.addData("Transfer (both)", transferOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Ball Block", blockOn ? "BLOCKING" : "OPEN");
            telemetry.addData("kS", FlywheelPIDF.kS);
            telemetry.addData("kV", FlywheelPIDF.kV);
            telemetry.addData("kA", FlywheelPIDF.kA);
            telemetry.addLine("A=Flywheel  B=Transfer  X=Intake  Y=Block  Dpad Up/Down=RPM");
            telemetry.update();
        }

        flywheelPID.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        transfer2.setPower(0);
        ballBlock.setPosition(BallBlockTest.BLOCK_MIN);
    }
}