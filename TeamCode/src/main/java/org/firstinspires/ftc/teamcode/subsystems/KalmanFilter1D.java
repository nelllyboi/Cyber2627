package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Simple one-number Kalman filter, used to smooth the flywheel RPM reading.
 *
 * Each loop:
 *   predict: the RPM probably stayed about the same, but we get less sure of it (P grows by Q)
 *   update:  blend in the new reading. K (0 to 1) is how much we trust the new reading.
 *
 * Q (process noise) = how much the real RPM can change per loop. Bigger Q = follows changes faster, less smoothing.
 * R (measurement noise) = how noisy the sensor is. Bigger R = more smoothing, more lag.
 * Only the ratio Q / R matters for how much it smooths.
 */
public class KalmanFilter1D {

    private double estimate = 0; // filtered value
    private double p = 1;        // how unsure we are about the estimate
    private double gain = 0;     // last Kalman gain K, for telemetry

    /** Feed a new reading, get the filtered value back. */
    public double update(double measurement, double q, double r) {
        p += q;                       // predict
        gain = p / (p + r);           // how much to trust the new reading
        estimate += gain * (measurement - estimate);
        p *= (1 - gain);
        return estimate;
    }

    /** Jump straight to a value (use when the flywheel starts, so it doesn't lag from 0). */
    public void reset(double value) {
        estimate = value;
        p = 1;
    }

    public double getEstimate() { return estimate; }
    public double getGain() { return gain; }
}
