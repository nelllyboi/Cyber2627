package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Robot heading from the goBILDA Pinpoint, for field-centric TeleOp driving.
 *
 * Talks to the Pinpoint directly (not through Pedro), so it works before
 * Pedro is tuned. If the Pinpoint isn't in the config, isAvailable() is false
 * and heading reads 0. Nothing crashes.
 *
 * zero() makes the current direction "forward" without resetting the IMU
 * (a reset needs the robot still and takes time).
 */
public class HeadingSensor {

    private GoBildaPinpointDriver pinpoint;
    private String error = null;

    private double offsetRad = 0;
    private double headingRad = 0;

    public HeadingSensor(HardwareMap hardwareMap) {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME);
        } catch (RuntimeException e) {
            pinpoint = null;
            error = e.getMessage();
        }
    }

    public boolean isAvailable() {
        return pinpoint != null;
    }

    /** Why the Pinpoint couldn't be found (null if it was found). */
    public String getError() {
        return error;
    }

    /** Read the Pinpoint. Call once per loop. */
    public void update() {
        if (pinpoint == null) return;
        pinpoint.update();
        headingRad = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /** Treat the direction the robot faces right now as heading 0. */
    public void zero() {
        offsetRad = headingRad;
    }

    /** Heading since the last zero(), counter-clockwise positive, wrapped to -pi..pi. */
    public double getHeadingRad() {
        return AngleUnit.normalizeRadians(headingRad - offsetRad);
    }
}
