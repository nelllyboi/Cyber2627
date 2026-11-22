package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Finds our alliance's hive AprilTags with the Limelight 3A and measures how far away they are.
 * This is a helper class, not an OpMode. Any OpMode can use it:
 *
 *   HiveVision vision = new HiveVision(hardwareMap, HiveVision.Alliance.RED);  // in init
 *   vision.update();                                                         // every loop
 *   if (vision.hasTarget()) { double d = vision.getDistanceInches(); }
 *   vision.stop();                                                           // when the OpMode ends
 *
 * Two distance methods:
 *   3D pose method (main): the Limelight's own solve of where the tag is in camera space,
 *       from how big and skewed the tag looks. Uses the tag size set in the Limelight pipeline
 *       (must be 82.55 mm), not the tag height, so it works whether the hive cell is raised or
 *       lowered. Needs full 3D turned on in the pipeline.
 *   Angle method (backup, comparison only): uses camera height, tag height and camera tilt.
 *       d = (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / tan(MOUNT_ANGLE_DEG + ty)
 *       The tags are on the bottom of each cell and the hive tilts, so one TAG_HEIGHT_IN is
 *       only right for one cell position. Never used for the shooter.
 *
 * All distances are horizontal (along the floor), in inches.
 * Tunable values live in {@link LimelightConstants}.
 */
public class HiveVision {

    /** ANY uses every AprilTag the camera sees, regardless of ID. Handy for testing. */
    public enum Alliance { RED, BLUE, ANY }

    private final Limelight3A limelight;
    private Alliance alliance;

    // Latest good reading from the chosen tag
    private boolean hasTarget = false;
    private int tagId = -1;
    private double txDegrees = 0;
    private double tyDegrees = 0;
    private double rawDistanceInches = Double.NaN;       // 3D pose method, this frame only
    private double smoothedDistanceInches = Double.NaN;  // 3D pose method, filtered
    private double angleDistanceInches = Double.NaN;     // angle method, NaN if unavailable
    private boolean tagSeenWithoutPose = false;          // our tag seen, but no 3D pose sent

    // Tag position relative to the camera lens, leveled to the floor (camera tilt removed), inches
    private double tagForwardInches = Double.NaN;  // straight ahead along the floor
    private double tagRightInches = Double.NaN;    // to the right (negative = left)
    private double tagUpInches = Double.NaN;       // above the lens

    // Timing
    private final ElapsedTime sinceLastGoodReading = new ElapsedTime();
    private boolean everHadReading = false;
    private long lastFrameTimestamp = -1;  // so the same Limelight frame isn't averaged in twice

    /**
     * Sets up and starts the Limelight. Safe to call in INIT: it only starts the camera,
     * nothing on the robot moves.
     */
    public HiveVision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, LimelightConstants.LIMELIGHT_NAME);
        limelight.setPollRateHz(LimelightConstants.POLL_RATE_HZ);
        limelight.pipelineSwitch(LimelightConstants.PIPELINE_INDEX);
        limelight.start();
    }

    /** Change alliance, e.g. when the driver picks it during INIT. Clears the current target. */
    public void setAlliance(Alliance alliance) {
        if (alliance != this.alliance) {
            this.alliance = alliance;
            clearTarget();
        }
    }

    /** Read the newest Limelight result. Call once per loop. */
    public void update() {
        // Lost: no good reading for too long. Clear it so smoothing restarts fresh next time.
        if (hasTarget && sinceLastGoodReading.milliseconds() > LimelightConstants.LOST_TIMEOUT_MS) {
            clearTarget();
        }

        LLResult result = limelight.getLatestResult();

        // Ignore results that are missing, invalid, from another pipeline, or too old
        if (result == null || !result.isValid()) return;
        if (result.getPipelineIndex() != LimelightConstants.PIPELINE_INDEX) return;
        if (result.getStaleness() > LimelightConstants.LOST_TIMEOUT_MS) return;

        // Same frame as last loop: nothing new to add
        if (result.getControlHubTimeStamp() == lastFrameTimestamp) return;
        lastFrameTimestamp = result.getControlHubTimeStamp();

        // Pick our alliance's tag with the biggest area (closest, most reliable)
        LLResultTypes.FiducialResult best = null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags != null) {
            for (LLResultTypes.FiducialResult tag : tags) {
                if (!isOurTag(tag.getFiducialId())) continue;
                if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                    best = tag;
                }
            }
        }
        if (best == null) return;

        // Our tag is in view but the pipeline isn't sending a 3D pose (full 3D off?)
        tagSeenWithoutPose = !readTagPosition(best.getTargetPoseCameraSpace());
        if (tagSeenWithoutPose) return;
        double distance = Math.hypot(tagRightInches, tagForwardInches);

        // Good reading
        tagId = best.getFiducialId();
        txDegrees = best.getTargetXDegrees();
        tyDegrees = best.getTargetYDegrees();
        rawDistanceInches = distance;
        angleDistanceInches = angleMethodDistance(best.getTargetYDegrees());

        // Exponential filter: new = alpha * reading + (1 - alpha) * old.
        // Right after (re)finding the target there is no "old", so start from the reading.
        if (!hasTarget || Double.isNaN(smoothedDistanceInches)) {
            smoothedDistanceInches = distance;
        } else {
            double a = LimelightConstants.SMOOTHING_ALPHA;
            smoothedDistanceInches = a * distance + (1 - a) * smoothedDistanceInches;
        }

        hasTarget = true;
        everHadReading = true;
        sinceLastGoodReading.reset();
    }

    /** Stop the Limelight. Call when the OpMode ends. */
    public void stop() {
        limelight.stop();
    }

    // --- Getters ---

    public boolean hasTarget() { return hasTarget; }

    /** Smoothed 3D-pose distance in inches. Use this for the shooter. NaN if no target. */
    public double getDistanceInches() { return hasTarget ? smoothedDistanceInches : Double.NaN; }

    /** 3D-pose distance from the latest frame only, no smoothing. NaN if no target. */
    public double getRawDistanceInches() { return hasTarget ? rawDistanceInches : Double.NaN; }

    /** Angle-method distance in inches, for comparison only. NaN if no target or the angle is too flat. */
    public double getAngleDistanceInches() { return hasTarget ? angleDistanceInches : Double.NaN; }

    /**
     * True if the last new frame had one of our tags but no 3D pose, so it couldn't be used.
     * Usually means full 3D is off in the Limelight pipeline.
     */
    public boolean isTagSeenWithoutPose() { return tagSeenWithoutPose; }

    /** Tag distance straight ahead of the camera, along the floor, inches. NaN if no target. */
    public double getTagForwardInches() { return hasTarget ? tagForwardInches : Double.NaN; }

    /** Tag distance to the right of the camera, inches (negative = left). NaN if no target. */
    public double getTagRightInches() { return hasTarget ? tagRightInches : Double.NaN; }

    /** Tag height above the camera lens, inches. NaN if no target. */
    public double getTagUpInches() { return hasTarget ? tagUpInches : Double.NaN; }

    /** Left/right angle to the tag in degrees, for aiming. Positive = tag is right of center. 0 if no target. */
    public double getTxDegrees() { return txDegrees; }

    /** Up/down angle to the tag in degrees. Positive = tag is above center. 0 if no target. */
    public double getTyDegrees() { return tyDegrees; }

    /** ID of the tag being used, or -1 if none. */
    public int getTagId() { return hasTarget ? tagId : -1; }

    /** Milliseconds since the last good reading, or -1 if there has never been one. */
    public double getMsSinceLastReading() {
        return everHadReading ? sinceLastGoodReading.milliseconds() : -1;
    }

    public Alliance getAlliance() { return alliance; }

    /** True if the Limelight is connected to the Control Hub. Handy for telemetry. */
    public boolean isConnected() { return limelight.isConnected(); }

    // --- Helpers ---

    private boolean isOurTag(int id) {
        if (LimelightConstants.ONLY_TAG_ID >= 0) return id == LimelightConstants.ONLY_TAG_ID;
        if (alliance == Alliance.ANY) return true;
        int[] ours = (alliance == Alliance.RED)
                ? LimelightConstants.RED_HIVE_TAG_IDS
                : LimelightConstants.BLUE_HIVE_TAG_IDS;
        for (int ourId : ours) {
            if (id == ourId) return true;
        }
        return false;
    }

    /**
     * Angle method. The camera is CAMERA_HEIGHT_IN off the floor and tilted up MOUNT_ANGLE_DEG.
     * The tag is ty degrees above the crosshair, so the total angle up to the tag is
     * MOUNT_ANGLE_DEG + ty. The height difference divided by tan(that angle) is the floor distance.
     * Returns NaN if the angle is too flat to give a sensible answer.
     */
    private double angleMethodDistance(double ty) {
        double totalAngleDeg = LimelightConstants.MOUNT_ANGLE_DEG + ty;
        if (Math.abs(totalAngleDeg) < LimelightConstants.MIN_TOTAL_ANGLE_DEG) return Double.NaN;

        double heightDiff = LimelightConstants.TAG_HEIGHT_IN - LimelightConstants.CAMERA_HEIGHT_IN;
        double distance = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));

        // A negative answer means the geometry constants don't match reality yet
        return distance > 0 ? distance : Double.NaN;
    }

    /**
     * 3D pose method. Limelight camera space: x = right, y = down, z = straight out of the lens.
     * The camera is tilted up, so the lens direction isn't level with the floor. Rotate by
     * MOUNT_ANGLE_DEG to get forward (along the floor), right and up.
     * Returns false if the pose is missing (the SDK fills a missing pose with all zeros).
     */
    private boolean readTagPosition(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) return false;

        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return false;

        double mount = Math.toRadians(LimelightConstants.MOUNT_ANGLE_DEG);
        tagForwardInches = p.z * Math.cos(mount) + p.y * Math.sin(mount);
        tagRightInches = p.x;
        tagUpInches = p.z * Math.sin(mount) - p.y * Math.cos(mount);
        return true;
    }

    private void clearTarget() {
        hasTarget = false;
        tagId = -1;
        txDegrees = 0;
        tyDegrees = 0;
        rawDistanceInches = Double.NaN;
        smoothedDistanceInches = Double.NaN;
        angleDistanceInches = Double.NaN;
        tagSeenWithoutPose = false;
        tagForwardInches = Double.NaN;
        tagRightInches = Double.NaN;
        tagUpInches = Double.NaN;
    }
}
