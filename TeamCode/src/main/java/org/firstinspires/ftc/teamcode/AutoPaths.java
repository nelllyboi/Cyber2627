package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto poses and paths (Pedro coordinates: inches, degrees in, radians stored).
 *
 * Named AutoPaths (not "Paths") so it doesn't clash with com.pedropathing.api.Paths.
 *
 * Every pose below is written for the RED alliance. For BLUE, the PoseFactory
 * mirrors each pose across the field center line (x = 72):
 *   x -> 144 - x,  heading -> 180 - heading
 * so there is only one set of numbers to maintain.
 */
public class AutoPaths {

    private static final double FIELD_CENTER_X = 72.0;  // half of the 144 in field

    // --- Red poses (x, y, heading in degrees) ---
    // Start: robot touching the back wall.
    private static final double[] START = {58.8627, 9.5336, 90};
    // Park: robot center ~6 in off the wall, front edge inside the loading zone.
    // Must NOT touch the wall at the end, or we lose LEAVE.
    private static final double[] PARK = {15, 110.5, 0};
    // Curve control point that keeps the path clear of the hive (heading unused).
    private static final double[] PARK_CONTROL = {32.3594, 26.6726, 0};

    private final Pose start;
    private final Pose park;
    private final Pose parkControl;

    public AutoPaths(boolean blue) {
        PoseFactory poses = blue
                ? PoseFactory.degrees().mirrorX(FIELD_CENTER_X)
                : PoseFactory.degrees();

        start = poses.of(START[0], START[1], START[2]);
        park = poses.of(PARK[0], PARK[1], PARK[2]);
        parkControl = poses.of(PARK_CONTROL[0], PARK_CONTROL[1], PARK_CONTROL[2]);
    }

    /** Start -> park as a curve through the control point, heading turning linearly. */
    public Path toPark() {
        return Paths.curve(start, parkControl, park).linear(start, park);
    }

    public Pose start() { return start; }
    public Pose park() { return park; }
}
