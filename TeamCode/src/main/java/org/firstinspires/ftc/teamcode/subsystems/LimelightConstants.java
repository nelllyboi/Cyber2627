package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Every tunable value for the Limelight 3A and hive distance detection.
 *
 * The non-final values show up in FTC Dashboard's
 * Configuration tab under LimelightConstants, so they can be adjusted live.
 * Changes made there are lost on restart: copy final values back into this file.
 *
 * Anything marked TODO is a placeholder that must be measured on the robot.
 * Distances will be wrong until they are.
 */
@Config
public class LimelightConstants {

    // --- Hardware / pipeline (final, not shown in Dashboard) ---
    public static final String LIMELIGHT_NAME = "limelight"; // must match the Driver Station config
    // The pipeline itself is set up in the Limelight web UI, not here:
    //   family 36h11, marker size 101.6 mm, full 3D turned ON.
    // The tags are really 82.55 mm (3.25 in), but 101.6 is what gave correct distances
    // on our robot in testing (82.55 read short, 165.1 read long). Treat it as a
    // calibration value: if distances drift, re-check against a tape measure
    // (floor, from under the lens to under the tag center) before changing it.
    // The marker size scales every 3D distance by the same factor.
    // "AprilTag size mismatch" in the web UI just means the uploaded field map (.fmap)
    // lists a different size. Our code doesn't use the field map, so it can be ignored.
    public static final int PIPELINE_INDEX    = 0;           // AprilTag (Fiducial) pipeline
    public static final int POLL_RATE_HZ      = 100;         // how often the SDK asks the Limelight for data

    // --- Camera and tag geometry ---
    // CAMERA_HEIGHT_IN and TAG_HEIGHT_IN are only used by the backup angle method.
    // MOUNT_ANGLE_DEG is used by both methods.

    // Floor to the CENTER of the Limelight lens, in inches.
    // Robot sitting flat on the field tiles, tape measure straight up.
    public static double CAMERA_HEIGHT_IN = 10.0;   // measured

    // TODO measure: floor to the CENTER of an AprilTag on the hive, in inches.
    // The tags are on the bottom face of each cell and the hive tilts, so this
    // changes with cell position. Only the backup angle method uses it.
    public static double TAG_HEIGHT_IN = 40.0;      // PLACEHOLDER

    // How far the camera is tilted up from horizontal, in degrees. Positive = tilted up.
    // Fine-tune: park at a tape-measured distance and adjust this until the
    // 3D pose distance matches the tape.
    public static double MOUNT_ANGLE_DEG = 40.0;    // measured

    // If (MOUNT_ANGLE_DEG + ty) is smaller than this, the tag is almost level
    // with the camera and tan() blows up, so the angle method is skipped.
    public static double MIN_TOTAL_ANGLE_DEG = 1.0;

    // --- Field tags (official BIOBUZZ tag files) ---
    // Red Scoring 30-33, Red Audience 34-37, Blue Audience 38-41, Blue Scoring 42-45.
    public static final int[] RED_HIVE_TAG_IDS  = {30, 31, 32, 33, 34, 35, 36, 37};
    public static final int[] BLUE_HIVE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    // If 0 or more, ONLY this tag is used and the alliance setting is ignored.
    // Set to -1 to go back to alliance filtering. 35 = a red audience cell tag.
    public static int ONLY_TAG_ID = 35;

    // --- Smoothing ---

    // Exponential filter weight for each new distance reading, 0 to 1.
    // Higher = reacts faster but jitters more. Lower = smoother but lags.
    // Tune by watching the smoothed distance on telemetry while standing still and moving.
    public static double SMOOTHING_ALPHA = 0.3;

    // No good reading for this long = target lost, and smoothing restarts fresh.
    public static double LOST_TIMEOUT_MS = 250;
}
