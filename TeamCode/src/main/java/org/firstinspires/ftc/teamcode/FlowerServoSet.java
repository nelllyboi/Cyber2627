package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Set both flower servos with ONE number from FTC Dashboard.
 *
 * Dashboard -> Configuration -> FlowerServoSet:
 *   BAR_POS        - the one number (0 to 1). flower1 goes to BAR_POS.
 *   MIRROR         - true:  flower2 goes to 1 - BAR_POS (servos face each other, the usual case)
 *                    false: flower2 goes to BAR_POS too (servos face the same way)
 *   FLOWER2_OFFSET - small fix added to flower2 if the bar twists a little (e.g. 0.02 or -0.02)
 *   FLOWER1_ON / FLOWER2_ON - false = that servo is limp. Both start false every run.
 *
 * SAFETY CHECK before turning both on (both servos are on one bar):
 *   1. Turn on ONLY flower1. Change BAR_POS up a bit. Note which way the bar moves.
 *   2. Turn it off, turn on ONLY flower2. Change BAR_POS up a bit again.
 *   3. The bar must move the SAME way both times. If it moves opposite ways, flip MIRROR.
 *   4. Only then turn both on. Press STOP right away if a servo buzzes or the bar twists.
 */
@Config
@TeleOp(name = "Flower Servo Set", group = "Test")
public class FlowerServoSet extends LinearOpMode {

    public static double BAR_POS = 0.5;
    public static boolean MIRROR = true;
    public static double FLOWER2_OFFSET = 0.0;
    public static boolean FLOWER1_ON = false;
    public static boolean FLOWER2_ON = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ServoImplEx flower1 = hardwareMap.get(ServoImplEx.class, Constants.FLOWER1_NAME);
        ServoImplEx flower2 = hardwareMap.get(ServoImplEx.class, Constants.FLOWER2_NAME);
        flower1.setPwmDisable();
        flower2.setPwmDisable();

        // Always start limp, even if the Dashboard switches were left on last time.
        FLOWER1_ON = false;
        FLOWER2_ON = false;

        telemetry.addLine("Set BAR_POS in Dashboard. Both servos start OFF (limp).");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double pos1 = clip(BAR_POS);
            double pos2 = clip((MIRROR ? 1 - BAR_POS : BAR_POS) + FLOWER2_OFFSET);

            set(flower1, FLOWER1_ON, pos1);
            set(flower2, FLOWER2_ON, pos2);

            telemetry.addData("BAR_POS", "%.3f", BAR_POS);
            telemetry.addData("Mirror", MIRROR);
            telemetry.addData("flower1", "%s  pos %.3f", FLOWER1_ON ? "ON " : "off", pos1);
            telemetry.addData("flower2", "%s  pos %.3f", FLOWER2_ON ? "ON " : "off", pos2);
            telemetry.addLine("Check each servo alone moves the bar the same way before turning both on.");
            telemetry.update();
        }
        flower1.setPwmDisable();
        flower2.setPwmDisable();
    }

    private static void set(ServoImplEx servo, boolean on, double pos) {
        if (on) {
            servo.setPwmEnable();
            servo.setPosition(pos);
        } else {
            servo.setPwmDisable();
        }
    }

    private static double clip(double v) { return Math.max(0, Math.min(1, v)); }
}
