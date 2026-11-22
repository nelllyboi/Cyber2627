package org.firstinspires.ftc.teamcode.noahswork;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FlywheelPIDF;
import org.firstinspires.ftc.teamcode.subsystems.HiveVision;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;

/**
 * Spins the outtake flywheel ("shooter", port 0) at an RPM picked from the Limelight distance.
 *
 * The distance comes from {@link HiveVision} (same tag filter as Hive Distance Test),
 * the RPM from {@link ShotTable}, and the speed control from {@link FlywheelPIDF}.
 * Nothing moves during INIT; only the Limelight camera starts.
 *
 * Controls (gamepad1, press Start + A to assign it):
 *   A            - toggle flywheel on/off
 *   Dpad Up/Down - trim the RPM up/down by TRIM_STEP_RPM (for finding table values)
 *   Y            - reset the trim to 0
 *
 * If the tag is lost, the flywheel keeps the last distance-based RPM.
 * If no tag has been seen yet, it uses NO_TARGET_RPM.
 *
 * ShotTable values are PLACEHOLDERS. Fill them in with real shots before relying on this.
 * Telemetry goes to the Driver Station and is mirrored to Panels (http://192.168.43.1:8001),
 * where targetRPM and currentRPM can be graphed.
 */
@Configurable
@TeleOp(name = "Auto RPM Shooter", group = "Test")
public class AutoRpmShooterTeleOp extends LinearOpMode {

    // --- Tunables (shown in Panels' Configurables tab) ---
    public static double NO_TARGET_RPM = 1000;  // used until the first tag is seen
    public static double TRIM_STEP_RPM = 50;    // RPM change per dpad press
    public static double RPM_TOLERANCE = 100;   // within this of target = "AT SPEED", ok to feed
    public static double MAX_RPM       = 5000;  // safety cap on the target RPM

    private TelemetryManager panels;
    private boolean panelsWorking = true;

    @Override
    public void runOpMode() {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.FLYWHEEL_NAME);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // same as TeleOpTest
        FlywheelPIDF flywheel = new FlywheelPIDF(hardwareMap, shooterMotor);

        HiveVision vision = new HiveVision(hardwareMap, HiveVision.Alliance.ANY);

        try {
            panels = PanelsTelemetry.INSTANCE.getTelemetry();
        } catch (Exception e) {
            panelsWorking = false;
        }

        boolean flywheelOn = false;
        double trimRPM = 0;
        double distanceRPM = NO_TARGET_RPM;  // last RPM picked from a real distance

        boolean prevA = false, prevUp = false, prevDown = false;

        // INIT: only the camera runs, so the driver can check the tag is seen
        while (opModeInInit()) {
            vision.update();
            show("Limelight connected", vision.isConnected() ? "Yes" : "No");
            show("Target found", vision.hasTarget() ? "Yes" : "No");
            show("Distance (in)", inches(vision.getDistanceInches()));
            telemetry.addLine("A = flywheel on/off, Dpad Up/Down = trim RPM, Y = reset trim");
            sendTelemetry();
        }

        while (opModeIsActive()) {
            vision.update();

            // --- Buttons (act once per press) ---
            if (gamepad1.a && !prevA) flywheelOn = !flywheelOn;
            if (gamepad1.dpad_up && !prevUp) trimRPM += TRIM_STEP_RPM;
            if (gamepad1.dpad_down && !prevDown) trimRPM -= TRIM_STEP_RPM;
            if (gamepad1.y) trimRPM = 0;
            prevA = gamepad1.a;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            // --- Pick the target RPM from the distance ---
            double distance = vision.getDistanceInches();  // smoothed, NaN if no tag
            if (!Double.isNaN(distance)) {
                distanceRPM = ShotTable.rpmForDistance(distance);
            }
            double targetRPM = Math.max(0, Math.min(MAX_RPM, distanceRPM + trimRPM));

            // --- Drive the flywheel ---
            // The flywheel only spins one way (power is never negative), so use the size of the
            // RPM. Depending on motor direction and encoder wiring, getRPM() can come out
            // negative, which made the controller think it was too slow and stay at full power.
            double rawRPM = flywheel.getRPM();
            double currentRPM = Math.abs(rawRPM);
            double power = 0;
            if (flywheelOn) {
                power = flywheel.controller(targetRPM, currentRPM);
                power = Math.max(0, Math.min(1, power));
            }
            flywheel.setPower(power);

            boolean atSpeed = flywheelOn && Math.abs(targetRPM - currentRPM) <= RPM_TOLERANCE;

            // --- Telemetry ---
            show("Flywheel", flywheelOn ? "ON" : "OFF");
            show("Ready to shoot", atSpeed ? "AT SPEED" : "not yet");
            show("Target found", vision.hasTarget() ? "Yes (tag " + vision.getTagId() + ")" : "No, holding last RPM");
            show("Distance (in)", inches(distance));
            show("Table RPM", String.format("%.0f", distanceRPM));
            show("Trim RPM", String.format("%+.0f", trimRPM));
            show("Target RPM", String.format("%.0f", targetRPM));
            show("Current RPM", String.format("%.0f", currentRPM));
            show("Raw encoder RPM", String.format("%.0f", rawRPM)); // sign check: negative is fine now
            show("Motor power", String.format("%.2f", power));
            show("Flywheel current (A)", String.format("%.2f", flywheel.getCurrent()));

            // For Panels' Graph widget
            graph("targetRPM", targetRPM);
            graph("currentRPM", currentRPM);
            graph("distance", distance);

            sendTelemetry();
        }

        flywheel.setPower(0);
        vision.stop();
    }

    // --- Telemetry helpers: Driver Station first, Panels as an extra ---

    private void show(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsWorking) panels.addData(caption, value);
    }

    private void graph(String name, double value) {
        if (panelsWorking && !Double.isNaN(value)) panels.addData(name, value);
    }

    private void sendTelemetry() {
        telemetry.update();
        if (panelsWorking) {
            try {
                panels.update();
            } catch (Exception e) {
                panelsWorking = false;
            }
        }
    }

    /** Distance as text, or "--" when there isn't one. */
    private static String inches(double d) {
        return Double.isNaN(d) ? "--" : String.format("%.1f", d);
    }
}
