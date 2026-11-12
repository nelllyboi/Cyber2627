package org.firstinspires.ftc.teamcode.noahswork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Diagnostic for the two transfer motors. Runs each one by itself and shows
 * which hub/port it is mapped to, so a dead motor can be traced to wiring vs config.
 *
 * Controls (gamepad1):
 *   Hold A       - run transferMotor only
 *   Hold B       - run transferMotor2 only
 *   Dpad Up/Down - raise / lower test power by POWER_STEP
 */
@TeleOp(name = "Transfer Motor Test", group = "Test")
public class TransferMotorTest extends LinearOpMode {

    private static final String TRANSFER_NAME  = "transferMotor";
    private static final String TRANSFER2_NAME = "transferMotor2";

    private static final double START_POWER = 0.3;
    private static final double POWER_STEP  = 0.1;
    private static final double MAX_POWER   = 1.0;

    @Override
    public void runOpMode() {
        DcMotorEx transfer  = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        DcMotorEx transfer2 = hardwareMap.get(DcMotorEx.class, TRANSFER2_NAME);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = START_POWER;
        boolean upPressed = false;
        boolean downPressed = false;

        telemetry.addData(TRANSFER_NAME, transfer.getConnectionInfo());
        telemetry.addData(TRANSFER2_NAME, transfer2.getConnectionInfo());
        telemetry.addLine("Hold A = transferMotor, hold B = transferMotor2");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upPressed) {
                power = Math.min(MAX_POWER, power + POWER_STEP);
            }
            if (gamepad1.dpad_down && !downPressed) {
                power = Math.max(0, power - POWER_STEP);
            }
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;

            transfer.setPower(gamepad1.a ? power : 0);
            transfer2.setPower(gamepad1.b ? power : 0);

            telemetry.addData("Test power", "%.1f", power);
            telemetry.addLine();
            telemetry.addData(TRANSFER_NAME + " port", transfer.getConnectionInfo());
            telemetry.addData(TRANSFER_NAME + " power", "%.2f", transfer.getPower());
            telemetry.addData(TRANSFER_NAME + " current (A)", "%.2f", transfer.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(TRANSFER_NAME + " velocity (ticks/s)", "%.0f", transfer.getVelocity());
            telemetry.addLine();
            telemetry.addData(TRANSFER2_NAME + " port", transfer2.getConnectionInfo());
            telemetry.addData(TRANSFER2_NAME + " power", "%.2f", transfer2.getPower());
            telemetry.addData(TRANSFER2_NAME + " current (A)", "%.2f", transfer2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(TRANSFER2_NAME + " velocity (ticks/s)", "%.0f", transfer2.getVelocity());
            telemetry.update();
        }

        transfer.setPower(0);
        transfer2.setPower(0);
    }
}
