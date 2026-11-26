package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Checks whether FTC Dashboard is running, and turns its saved "auto enable"
 * setting back on.
 *
 * FTC Dashboard remembers being disabled across restarts (saved setting
 * "autoEnable" in the "FtcDashboard" preferences). Normally you'd flip it with
 * the "Enable/Disable Dashboard" OpMode, but if that OpMode is missing, run this:
 *   INIT  - shows whether Dashboard is running and its saved setting
 *   START - sets the saved setting back to enabled
 * Then restart the robot (power cycle) so Dashboard starts enabled.
 *
 * No motors or servos are used.
 */
@TeleOp(name = "Dashboard Check", group = "Test")
public class DashboardCheck extends LinearOpMode {

    // Must match FtcDashboard 0.5.1's internal preference names.
    private static final String PREFS_NAME = "FtcDashboard";
    private static final String AUTO_ENABLE_KEY = "autoEnable";

    @Override
    public void runOpMode() {
        Activity activity = AppUtil.getInstance().getActivity();
        SharedPreferences prefs = activity.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addData("Dashboard running", dashboard != null ? "YES" : "NO (library not started)");
        telemetry.addData("Dashboard enabled now", dashboard != null && dashboard.isEnabled());
        telemetry.addData("Saved auto-enable", prefs.getBoolean(AUTO_ENABLE_KEY, true));
        telemetry.addLine("Press START to set auto-enable back on, then restart the robot.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        prefs.edit().putBoolean(AUTO_ENABLE_KEY, true).commit();

        telemetry.addData("Saved auto-enable", prefs.getBoolean(AUTO_ENABLE_KEY, true));
        telemetry.addLine("Done. Power cycle the robot, then open 192.168.43.1:8080/dash");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
