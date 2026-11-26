package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Turns a Limelight distance into a flywheel target RPM.
 *
 * The table is a list of (distance, RPM) pairs measured on the real robot.
 * Between two rows the RPM is linearly interpolated. Closer than the first row
 * uses the first RPM, farther than the last row uses the last RPM, so the
 * flywheel is never asked for something outside what was tested.
 *
 * How to fill in the table (with the "Auto RPM Shooter" TeleOp):
 *   1. Park at a distance, spin up, and shoot.
 *   2. Use dpad up/down to trim the RPM until shots go in reliably.
 *   3. Write down "Distance (in)" and "Target RPM" from telemetry as one row.
 *   4. Repeat at 4-6 distances across your shooting range, closest first.
 *
 * Both arrays show up in FTC Dashboard's Configuration tab for live editing, but copy
 * final values back here so they survive a restart.
 */
@Config
public class ShotTable {

    // TODO measure: PLACEHOLDER values. Replace every row with real test data.
    // Distances in inches (camera to hive tag, along the floor), smallest first.
    public static double[] DISTANCES_IN = {40, 60, 80, 100, 120};

    // TODO measure: PLACEHOLDER values. Flywheel RPM for each distance above.
    public static double[] RPMS = {2000, 2400, 2800, 3200, 3600};

    private ShotTable() {}

    /** Target RPM for this distance, interpolated from the table. */
    public static double rpmForDistance(double distanceInches) {
        int rows = Math.min(DISTANCES_IN.length, RPMS.length);
        if (rows == 0) return 0;

        if (distanceInches <= DISTANCES_IN[0]) return RPMS[0];
        if (distanceInches >= DISTANCES_IN[rows - 1]) return RPMS[rows - 1];

        // Find the two rows on either side and blend between them
        for (int i = 1; i < rows; i++) {
            if (distanceInches <= DISTANCES_IN[i]) {
                double d0 = DISTANCES_IN[i - 1], d1 = DISTANCES_IN[i];
                double r0 = RPMS[i - 1], r1 = RPMS[i];
                double t = (distanceInches - d0) / (d1 - d0);
                return r0 + t * (r1 - r0);
            }
        }
        return RPMS[rows - 1];
    }
}
