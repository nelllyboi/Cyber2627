package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel feedforward tuning test for Cyber2627.
 *
 * Ported from the decode-2025-26 "Feedforward Increments" OpMode. The flywheel
 * control math lives in {@link FlywheelPIDF}. This repo has no turret, so no
 * turret / targeting code is included. FtcDashboard was removed so this runs on
 * the stock FTC SDK with no extra dependencies — tuning is done over the normal
 * Driver Station telemetry.
 *
 * Tuning notes (edit the gains in FlywheelPIDF):
 *   kS = minimum power to make the flywheel start spinning  -> tune FIRST
 *   kV = power per RPM, gets you to the target RPM           -> tune SECOND
 *   kA = extra power while the target RPM is changing        -> tune LAST
 *
 * Hardware config names (rename the constants below to match your Driver Station):
 *   "port0Motor"    - flywheel motor
 *   "transferMotor" - transfer/feeder motor
 *   "intakeMotor"   - intake motor
 *
 * Controls (gamepad1):
 *   A            - toggle flywheel on/off
 *   B            - toggle transfer motor on/off
 *   X            - toggle intake motor on/off
 *   Dpad Up/Down - raise / lower target RPM by RPM_STEP
 */
@TeleOp(name = "Feedforward Increments", group = "Test")
public class FeedforwardTest extends LinearOpMode {

    // --- Hardware config names ---
    private static final String FLYWHEEL_NAME = "shooter";
    private static final String TRANSFER_NAME = "transferMotor";
    private static final String INTAKE_NAME   = "intakeMotor";

    // --- Tuning ---
    private static final double RPM_STEP      = 100;   // RPM change per dpad press
    private static final double START_RPM     = 1000;  // initial target RPM
    private static final double INTAKE_POWER  = -1.0;
    private static final double TRANSFER_POWER = 1.0;

    private DcMotorEx flywheelMotor;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private FlywheelPIDF flywheelPID;

    private double targetRPM = START_RPM;

    private boolean flywheelOn = false;
    private boolean transferOn = false;
    private boolean intakeOn = false;

    // Rising-edge state
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean dUPressed = false;
    private boolean dDPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Single-motor flywheel. For a dual-motor flywheel, fetch a second
        // DcMotorEx and use: new FlywheelPIDF(hardwareMap, flywheelMotor, flywheelMotor2);
        flywheelPID = new FlywheelPIDF(hardwareMap, flywheelMotor);

        telemetry.addLine("Ready. A = flywheel, B = transfer, X = intake, Dpad = RPM.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double currentRPM = flywheelPID.getRPM();
            double power = 0;

            // Transfer motor on/off
            if (transferOn) {
                transfer.setPower(TRANSFER_POWER);
            } else {
                transfer.setPower(0);
            }

            // Intake motor on/off
            if (intakeOn) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // A: toggle flywheel
            if (gamepad1.a) {
                if (!aPressed) {
                    flywheelOn = !flywheelOn;
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            // B: toggle transfer
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

            telemetry.addData("Flywheel Active", flywheelOn ? "Yes" : "No");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", "%.1f", currentRPM);
            telemetry.addData("Error RPM", "%.1f", flywheelOn ? (targetRPM - currentRPM) : 0);
            telemetry.addData("Motor Power", "%.3f", power);
            telemetry.addData("Flywheel Current (A)", "%.2f", flywheelPID.getCurrent());
            telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("kS", FlywheelPIDF.kS);
            telemetry.addData("kV", FlywheelPIDF.kV);
            telemetry.addData("kA", FlywheelPIDF.kA);
            telemetry.addLine("A=Flywheel  B=Transfer  X=Intake  Dpad Up/Down=RPM");
            telemetry.update();
        }

        flywheelPID.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}