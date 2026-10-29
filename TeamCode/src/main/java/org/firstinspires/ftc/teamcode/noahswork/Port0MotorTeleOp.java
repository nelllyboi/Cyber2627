package org.firstinspires.ftc.teamcode.noahswork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Turns on the motor in port 0, the transfer motor in port 1, and the intake motor in port 2.
 * The Driver Station config must name them MOTOR_NAME, TRANSFER_MOTOR_NAME and INTAKE_MOTOR_NAME.
 *
 * Controls (gamepad1):
 *   A - toggle port 0 motor on/off
 *   B - toggle transfer motor (port 1) on/off
 *   X - toggle intake motor (port 2) on/off
 */
@TeleOp(name = "Port 0 Motor", group = "Test")
public class Port0MotorTeleOp extends LinearOpMode {

    private static final String MOTOR_NAME = "port0Motor";
    private static final double MOTOR_POWER = 1.0; // start low; raise once verified

    private static final String TRANSFER_MOTOR_NAME = "transferMotor";
    private static final double TRANSFER_POWER = 1.0;

    private static final String INTAKE_MOTOR_NAME = "intakeMotor";
    private static final double INTAKE_POWER = -1.0;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_MOTOR_NAME);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        boolean motorOn = false;
        boolean prevA = false;
        boolean transferOn = false;
        boolean prevTransferButton = false;
        boolean intakeOn = false;
        boolean prevX = false;

        telemetry.addLine("Ready. Use GAMEPAD 1 (Start + A to assign it).");
        telemetry.addLine("A = port 0 motor, B = transfer motor, X = intake motor.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean currA = gamepad1.a;
            if (currA && !prevA) {
                motorOn = !motorOn;
            }
            prevA = currA;

            boolean currTransferButton = gamepad1.b;
            if (currTransferButton && !prevTransferButton) {
                transferOn = !transferOn;
            }
            prevTransferButton = currTransferButton;

            boolean currX = gamepad1.x;
            if (currX && !prevX) {
                intakeOn = !intakeOn;
            }
            prevX = currX;

            double power = motorOn ? MOTOR_POWER : 0.0;
            motor.setPower(power);

            double transferPower = transferOn ? TRANSFER_POWER : 0.0;
            transfer.setPower(transferPower);

            double intakePower = intakeOn ? INTAKE_POWER : 0.0;
            intake.setPower(intakePower);

            telemetry.addLine("Controller: GAMEPAD 1");
            telemetry.addData("Transfer button held", currTransferButton);
            telemetry.addData("Motor", motorOn ? "ON" : "OFF");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Velocity (tks/s)", "%.1f", motor.getVelocity());
            telemetry.addData("Port", motor.getPortNumber());
            telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
            telemetry.addData("Transfer Power", "%.2f", transferPower);
            telemetry.addData("Transfer Velocity (tks/s)", "%.1f", transfer.getVelocity());
            telemetry.addData("Transfer Port", transfer.getPortNumber());
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Intake Velocity (tks/s)", "%.1f", intake.getVelocity());
            telemetry.addData("Intake Port", intake.getPortNumber());
            telemetry.update();
        }

        motor.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
    }
}
