package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Final TeleOp for Cyber2627.
 *
 * Controls (gamepad1):
 *   A - ball block to BLOCK_POS_A (0.47)
 *   B - ball block to BLOCK_POS_B (0.7)
 *   Y - toggle outtake flywheel on/off (runs at OUTTAKE_RPM using FlywheelPIDF)
 *
 * Nothing moves during INIT (G403). The ball block servo is not commanded
 * until A or B is pressed.
 */
@Config
@TeleOp(name = "Final TeleOp", group = "Competition")
public class FinalTeleOp extends LinearOpMode {

    // --- Hardware config names ---
    private static final String FLYWHEEL_NAME   = "shooter";
    private static final String BALL_BLOCK_NAME = BallBlockTest.BALL_BLOCK_NAME;

    // --- Tunables (editable in FTC Dashboard) ---
    public static double BLOCK_POS_A = 0.47;
    public static double BLOCK_POS_B = 0.7;
    public static double OUTTAKE_RPM = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FlywheelPIDF flywheel = new FlywheelPIDF(hardwareMap, flywheelMotor);

        Servo ballBlock = hardwareMap.get(Servo.class, BALL_BLOCK_NAME);

        boolean outtakeOn = false;
        boolean yWasPressed = false;
        Double blockPos = null;  // null = not commanded yet

        telemetry.addLine("Ready. A/B = ball block, Y = outtake on/off.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Ball block presets
            if (gamepad1.a) {
                blockPos = BLOCK_POS_A;
            } else if (gamepad1.b) {
                blockPos = BLOCK_POS_B;
            }
            if (blockPos != null) {
                ballBlock.setPosition(blockPos);
            }

            // Y: toggle outtake on the press (rising edge)
            if (gamepad1.y && !yWasPressed) {
                outtakeOn = !outtakeOn;
            }
            yWasPressed = gamepad1.y;

            // Drive the outtake flywheel
            double currentRPM = flywheel.getRPM();
            double power = 0;
            if (outtakeOn) {
                power = flywheel.controller(OUTTAKE_RPM, currentRPM);
                power = Math.max(0, Math.min(1, power));
            }
            flywheel.setPower(power);

            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", OUTTAKE_RPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Ball Block Pos", blockPos == null ? "not set" : blockPos);
            telemetry.addLine("A=Block 0.47  B=Block 0.7  Y=Outtake on/off");
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}
