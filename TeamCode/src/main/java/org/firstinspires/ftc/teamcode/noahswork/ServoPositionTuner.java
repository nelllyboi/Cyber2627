package org.firstinspires.ftc.teamcode.noahswork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Find servo positions with the gamepad, no FTC Dashboard needed.
 * The servo does not move during INIT. It goes to START_POS when START is pressed.
 *
 * Controls (gamepad1):
 *   Dpad Up/Down     - position +/- BIG_STEP
 *   Right/Left bumper - position +/- SMALL_STEP
 *   A                - jump back to START_POS
 *
 * Read the position off the Driver Station and write down the open and blocking values.
 */
@TeleOp(name = "Servo Position Tuner", group = "Test")
public class ServoPositionTuner extends LinearOpMode {

    private static final String SERVO_NAME = "ballBlock";

    private static final double START_POS  = 0.5;
    private static final double BIG_STEP   = 0.05;
    private static final double SMALL_STEP = 0.01;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);

        double pos = START_POS;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean rbPressed = false;
        boolean lbPressed = false;

        telemetry.addData("Servo", SERVO_NAME);
        telemetry.addLine("Press START. Servo will go to " + START_POS);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upPressed)       pos += BIG_STEP;
            if (gamepad1.dpad_down && !downPressed)   pos -= BIG_STEP;
            if (gamepad1.right_bumper && !rbPressed)  pos += SMALL_STEP;
            if (gamepad1.left_bumper && !lbPressed)   pos -= SMALL_STEP;
            if (gamepad1.a)                           pos = START_POS;

            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;
            rbPressed = gamepad1.right_bumper;
            lbPressed = gamepad1.left_bumper;

            pos = Math.max(0.0, Math.min(1.0, pos));
            servo.setPosition(pos);

            telemetry.addData("Position", "%.2f", pos);
            telemetry.addLine("Dpad Up/Down = +/-" + BIG_STEP + "   RB/LB = +/-" + SMALL_STEP + "   A = " + START_POS);
            telemetry.update();
        }
    }
}
