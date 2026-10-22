package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Test")
public class SingleMotorTest extends LinearOpMode {

    private static final String MOTOR_NAME  = "testMotor";
    private static final double DEADZONE    = 0.05;
    private static final double POWER_STEP  = 0.1;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        double fixedPower = 0.5;

        boolean prevX        = false;
        boolean prevDpadUp   = false;
        boolean prevDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean currX        = gamepad1.x;
            boolean currDpadUp   = gamepad1.dpad_up;
            boolean currDpadDown = gamepad1.dpad_down;

            // Rising-edge: toggle direction
            if (currX && !prevX) {
                direction = (direction == DcMotorSimple.Direction.FORWARD)
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD;
                motor.setDirection(direction);
            }

            // Rising-edge: adjust fixed power, clamped [0, 1]
            if (currDpadUp && !prevDpadUp) {
                fixedPower = Math.min(1.0, Math.round((fixedPower + POWER_STEP) * 10.0) / 10.0);
            }
            if (currDpadDown && !prevDpadDown) {
                fixedPower = Math.max(0.0, Math.round((fixedPower - POWER_STEP) * 10.0) / 10.0);
            }

            // Power priority: B (stop) > A (fixed) > left stick
            double power;
            if (gamepad1.b) {
                power = 0.0;
            } else if (gamepad1.a) {
                power = fixedPower;
            } else {
                double stick = -gamepad1.left_stick_y; // negate: up = positive
                power = Math.abs(stick) < DEADZONE ? 0.0 : stick;
            }

            motor.setPower(power);

            telemetry.addData("Power",            "%.2f", power);
            telemetry.addData("Direction",         direction);
            telemetry.addData("Encoder Position",  motor.getCurrentPosition());
            telemetry.addData("Velocity (tks/s)", "%.1f", motor.getVelocity());
            telemetry.addData("Fixed Test Power",  "%.1f", fixedPower);
            telemetry.update();

            prevX        = currX;
            prevDpadUp   = currDpadUp;
            prevDpadDown = currDpadDown;
        }

        motor.setPower(0);
    }
}
