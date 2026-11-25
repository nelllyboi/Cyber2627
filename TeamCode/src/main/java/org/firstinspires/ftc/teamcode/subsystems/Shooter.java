package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FlywheelPIDF;

/**
 * Outtake flywheel ("shooter") plus the ball block servo in front of it.
 *
 * The flywheel uses the team's FlywheelPIDF controller (PID + kS/kV/kA feedforward,
 * gains tunable in FTC Dashboard). On top of that, the power is scaled by
 * NOMINAL_VOLTAGE / battery voltage so a low battery still reaches the same RPM.
 *
 * Call update() once every loop. Nothing moves until you call a method:
 * creating a Shooter does not spin the motor or move the servo.
 */
public class Shooter {

    private final FlywheelPIDF flywheel;
    private final Servo ballBlock;
    private final VoltageSensor battery;

    // Times how long the flywheel has been continuously at speed.
    private final ElapsedTime atSpeedTimer = new ElapsedTime();
    private boolean wasAtSpeed = false;

    private double targetRpm = 0;
    private double currentRpm = 0;
    private double power = 0;

    public Shooter(HardwareMap hardwareMap) {
        // Same motor setup as TeleOpTest, so RPM readings match.
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Constants.FLYWHEEL_NAME);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel = new FlywheelPIDF(hardwareMap, motor);

        ballBlock = hardwareMap.get(Servo.class, Constants.BALL_BLOCK_NAME);
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /** Target flywheel speed. 0 turns the flywheel off. */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void stop() {
        setTargetRpm(0);
    }

    /** Run the velocity controller. Call once per loop. */
    public void update() {
        currentRpm = flywheel.getRPM();

        if (targetRpm <= 0) {
            power = 0;
        } else {
            double voltageScale = Constants.NOMINAL_VOLTAGE / Math.max(battery.getVoltage(), 1.0);
            power = flywheel.controller(targetRpm, currentRpm) * voltageScale;
            power = Math.max(0, Math.min(1, power));
        }
        flywheel.setPower(power);

        // Restart the "at speed" timer whenever the flywheel falls out of tolerance.
        boolean atSpeed = isAtSpeed();
        if (atSpeed && !wasAtSpeed) {
            atSpeedTimer.reset();
        }
        wasAtSpeed = atSpeed;
    }

    /** RPM is within RPM_TOLERANCE of the target right now. */
    public boolean isAtSpeed() {
        return targetRpm > 0 && Math.abs(targetRpm - currentRpm) <= Constants.RPM_TOLERANCE;
    }

    /** At speed and has stayed there for READY_HOLD_SEC. Safe to feed a ball. */
    public boolean isReady() {
        return wasAtSpeed && atSpeedTimer.seconds() >= Constants.READY_HOLD_SEC;
    }

    /** RPM has dropped far enough below target that a ball just went through. */
    public boolean dipDetected() {
        return targetRpm > 0 && currentRpm < targetRpm - Constants.SHOT_DIP_RPM;
    }

    public void openBallBlock() {
        ballBlock.setPosition(Constants.BALL_BLOCK_OPEN);
    }

    public void closeBallBlock() {
        ballBlock.setPosition(Constants.BALL_BLOCK_CLOSED);
    }

    public double getTargetRpm() { return targetRpm; }
    public double getCurrentRpm() { return currentRpm; }
    public double getPower() { return power; }
}
