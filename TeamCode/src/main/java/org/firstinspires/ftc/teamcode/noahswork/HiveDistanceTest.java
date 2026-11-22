package org.firstinspires.ftc.teamcode.noahswork;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HiveVision;

/**
 * Test for Limelight hive distance detection. Nothing on the robot moves.
 *
 * Only uses LimelightConstants.ONLY_TAG_ID (35). Set it to -1 to use any tag the camera sees.
 * The Limelight already runs during INIT, so you can check that tags are seen before START.
 *
 * Compare "3D pose distance" against a tape measure from the camera to the hive tag
 * (along the floor) to check LimelightConstants. "Angle distance" is a backup shown
 * for comparison only.
 *
 * Telemetry goes to the Driver Station and is mirrored to Panels (http://192.168.43.1:8001).
 * In Panels' Graph widget these numbers can be plotted live:
 *   poseDistance, angleDistance   - the two distance methods, to compare
 *   tagForward, tagRight, tagUp   - where the tag is relative to the camera, in inches
 * LimelightConstants can be tuned in Panels' Configurables tab.
 */
@TeleOp(name = "Hive Distance Test", group = "Test")
public class HiveDistanceTest extends LinearOpMode {

    private TelemetryManager panels;
    private boolean panelsWorking = true;  // turned off if Panels ever throws, so the test keeps running

    @Override
    public void runOpMode() {
        HiveVision vision = new HiveVision(hardwareMap, HiveVision.Alliance.ANY);
        try {
            panels = PanelsTelemetry.INSTANCE.getTelemetry();
        } catch (Exception e) {
            panelsWorking = false;
        }

        // The Limelight is only a camera, so updating it in INIT is fine.
        while (opModeInInit()) {
            vision.update();

            show("Limelight connected", vision.isConnected() ? "Yes" : "No");
            show("Target found", vision.hasTarget() ? "Yes" : "No");
            show("Tag ID", vision.getTagId());
            addNoPoseWarning(vision);
            sendTelemetry();
        }

        while (opModeIsActive()) {
            vision.update();

            show("Target found", vision.hasTarget() ? "Yes" : "No");
            show("Tag ID", vision.getTagId());
            show("tx (deg)", String.format("%.2f", vision.getTxDegrees()));
            show("ty (deg)", String.format("%.2f", vision.getTyDegrees()));
            show("3D pose distance (in)", inches(vision.getRawDistanceInches()));
            show("Smoothed distance (in)", inches(vision.getDistanceInches()));
            show("Tag forward (in)", inches(vision.getTagForwardInches()));
            show("Tag right (in)", inches(vision.getTagRightInches()));
            show("Tag up (in)", inches(vision.getTagUpInches()));
            show("Angle distance, backup (in)", inches(vision.getAngleDistanceInches()));
            addNoPoseWarning(vision);
            double ms = vision.getMsSinceLastReading();
            show("Since last reading (ms)", ms < 0 ? "never" : String.format("%.0f", ms));

            // Numbers for Panels' Graph widget. Graph reads lines shaped "name: number",
            // so these use short names with no spaces and are skipped when there is no reading.
            graph("poseDistance", vision.getRawDistanceInches());
            graph("angleDistance", vision.getAngleDistanceInches());
            graph("tagForward", vision.getTagForwardInches());
            graph("tagRight", vision.getTagRightInches());
            graph("tagUp", vision.getTagUpInches());

            sendTelemetry();
        }

        vision.stop();
    }

    // --- Telemetry helpers: Driver Station first, Panels as an extra ---

    /** One line on the Driver Station and in Panels. */
    private void show(String caption, Object value) {
        telemetry.addData(caption, value);
        if (panelsWorking) panels.addData(caption, value);
    }

    /** A number for Panels' Graph widget only, skipped when there is no reading (NaN). */
    private void graph(String name, double value) {
        if (panelsWorking && !Double.isNaN(value)) panels.addData(name, value);
    }

    /**
     * Sends this loop's lines. The Driver Station is updated on its own, so a problem
     * in Panels can never freeze or blank the Driver Station.
     */
    private void sendTelemetry() {
        if (!panelsWorking) telemetry.addLine("Panels not working, Driver Station only");
        telemetry.update();
        if (panelsWorking) {
            try {
                panels.update();
            } catch (Exception e) {
                panelsWorking = false;
            }
        }
    }

    private void addNoPoseWarning(HiveVision vision) {
        if (vision.isTagSeenWithoutPose()) {
            String warning = "Tag seen but no 3D pose: turn on full 3D in the Limelight pipeline";
            telemetry.addLine(warning);
            if (panelsWorking) panels.addLine(warning);
        }
    }

    /** Distance as text, or "--" when there isn't one. */
    private static String inches(double d) {
        return Double.isNaN(d) ? "--" : String.format("%.1f", d);
    }
}
