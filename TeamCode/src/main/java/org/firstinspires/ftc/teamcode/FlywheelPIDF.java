package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Closed-loop flywheel controller: PID + feedforward (kS, kV, kA).
 *
 * Ported from the decode-2025-26 flywheel controller for Cyber2627.
 * All public static gains below show up in FTC Dashboard's Configuration tab
 * (under FlywheelPIDF) and can be edited live while the flywheel runs.
 * Copy final values back into the code so they survive an app restart.
 *
 * Feedforward meaning:
 *   kS = static power: the minimum power needed to overcome friction and start moving
 *   kV = velocity gain: power per unit of target RPM (behaves like the main "P")
 *   kA = acceleration gain: extra power applied while the target RPM is changing
 *
 * Suggested tuning order: kS first, then kV, then kA.
 *
 * Supports one or two flywheel motors (dual motors run at the same power).
 */
@Config
public class FlywheelPIDF {

    private static final double MAX_INTEGRAL_SUM = 100;

    // Encoder ticks per FLYWHEEL revolution, so getRPM() is the real flywheel speed.
    // goBILDA 1620 RPM motor: 28 ticks per motor rev x 3.7 gearbox = 103.8 ticks per output rev,
    // then geared up 3x to the flywheel: 103.8 / 3 = 34.6 ticks per flywheel rev.
    public static double TICKS_PER_REV = 34.6;

    public double wheelRad;

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2; // may be null for a single-motor flywheel

    private final ElapsedTime timer = new ElapsedTime();

    // PID state
    private double lastError = 0;
    private double integralSum = 0;

    // Feedforward state
    private double lastReferenceRPM = 0;

    // PID gains
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Feedforward gains
    public static double kS = 0.315475;   // static friction (power), Flywheel Characterize 2026-11-26 after dead-spot fix
    public static double kV = 0.00027061; // velocity feedforward (power / flywheel RPM), same run, R² 0.9979
    public static double kA = 0;              // acceleration feedforward (per-loop change in target RPM)

    /** Dual-motor flywheel. */
    public FlywheelPIDF(HardwareMap hardwareMap, DcMotorEx motor, DcMotorEx motor2) {
        flywheel = motor;
        flywheel2 = motor2;

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (flywheel2 != null) {
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /** Single-motor flywheel. */
    public FlywheelPIDF(HardwareMap hardwareMap, DcMotorEx motor) {
        this(hardwareMap, motor, null);
    }

    private double sign(double x) {
        if (x > 0) return 1.0;
        if (x < 0) return -1.0;
        return 0.0;
    }

    /**
     * Compute the motor power for the given target RPM.
     *
     * @param referenceRPM desired flywheel RPM
     * @param currentRPM   measured flywheel RPM (from {@link #getRPM()})
     * @return raw power command (clamp to [0, 1] before applying)
     */
    public double controller(double referenceRPM, double currentRPM) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) {
            dt = 1e-3;
        }

        double error = referenceRPM - currentRPM;

        integralSum += error * dt;
        integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = (kP * error) + (kI * integralSum) + (kD * derivative);

        double ksTerm = (referenceRPM != 0) ? kS * sign(referenceRPM) : 0;
        double kvTerm = kV * referenceRPM;

        double referenceAccel = referenceRPM - lastReferenceRPM;
        lastReferenceRPM = referenceRPM;

        double kaTerm = kA * referenceAccel;

        return pid + ksTerm + kvTerm + kaTerm;
    }

    public void setPower(double power) {
        flywheel.setPower(power);
        if (flywheel2 != null) {
            flywheel2.setPower(power);
        }
    }

    public double getCurrent() {
        if (flywheel2 != null) {
            return (flywheel.getCurrent(CurrentUnit.AMPS)
                    + flywheel2.getCurrent(CurrentUnit.AMPS)) / 2.0;
        }
        return flywheel.getCurrent(CurrentUnit.AMPS);
    }

    /** Raw encoder velocity in ticks per second (same sign as getRPM()). */
    public double getTicksPerSecond() {
        return -flywheel.getVelocity();
    }

    /** Raw encoder position in ticks (counts since the hub started). */
    public int getEncoderPosition() {
        return flywheel.getCurrentPosition();
    }

    public double getRPM() {
        double velocityTicksPerSecond = -flywheel.getVelocity();
        return (velocityTicksPerSecond / TICKS_PER_REV) * 60.0;
    }

    public double eVelToRPM(double eVel) {
        return eVel / wheelRad;
    }
}