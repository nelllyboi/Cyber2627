package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Ball block servo tuning for Cyber2627.
 *
 * Open FTC Dashboard (http://192.168.43.1:8080/dash), go to Configuration, and
 * edit BALL_BLOCK_POS under BallBlockTest. The servo moves live. Find the open
 * position and the blocking position, then write them into BLOCK_MIN and
 * BLOCK_MAX below. TeleOpTest reads those two constants.
 *
 * Gamepad 1: A moves the servo to BUTTON_A_POS (0.47), B to BUTTON_B_POS (0.7).
 */
@Config
@TeleOp(name = "Ball Block Test", group = "Test")
public class BallBlockTest extends OpMode {

    // --- Hardware config name ---
    public static final String BALL_BLOCK_NAME = "ballBlock";

    // --- Saved positions used by TeleOpTest ---
    public static double BLOCK_MIN = 0.34;   // blocking position
    public static double BLOCK_MAX = 0.5;   // open position

    // --- Gamepad preset positions ---
    public static double BUTTON_A_POS = 0.47;
    public static double BUTTON_B_POS = 0.7;

    // --- Live tuning position (edit in Dashboard, or set with A/B) ---
    public static double BALL_BLOCK_POS = 0.0;

    private Servo ballBlockServo;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        ballBlockServo = hardwareMap.get(Servo.class, BALL_BLOCK_NAME);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a) {
            BALL_BLOCK_POS = BUTTON_A_POS;
        } else if (gamepad1.b) {
            BALL_BLOCK_POS = BUTTON_B_POS;
        }

        ballBlockServo.setPosition(BALL_BLOCK_POS);

        telemetry.addData("Ball Block Position", BALL_BLOCK_POS);
        packet.put("Ball Block Pos", BALL_BLOCK_POS);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}