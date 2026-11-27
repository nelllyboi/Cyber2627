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
    private String mode = "OFF"; // OFF, BANG-BANG or PIDF
    private boolean bangActive = false; // remembered between loops for the enter/exit thresholds

    // Shot meter state (see measureShot)
    private final ElapsedTime shotTimer = new ElapsedTime();
    private int shotPhase = 0; // 0 waiting, 1 recovering, 2 watching overshoot
    private double shotLowest = 0, shotRecoverSec = 0, shotOvershoot = 0;
    private double lastLowestRpm = 0, lastRecoverSec = 0, lastOvershootRpm = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double shotBangSec = 0, lastBangSec = 0; // time spent at full power during a shot
    private int bangTriggers = 0;                    // times bang-bang switched on since START
    private int shotsMeasured = 0;

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
            mode = "OFF";
            bangActive = false;
        } else {
            // Always run the PIDF so its timer and error history stay current.
            double voltageScale = Constants.NOMINAL_VOLTAGE / Math.max(battery.getVoltage(), 1.0);
            double pidfPower = flywheel.controller(targetRpm, currentRpm) * voltageScale;

            // Two thresholds: turn on below ENTER, turn off once within EXIT.
            double error = targetRpm - currentRpm;
            if (!Constants.USE_BANG_BANG) {
                bangActive = false;
            } else if (error > Constants.BANG_BANG_ENTER_RPM) {
                bangActive = true;
            } else if (error < Constants.BANG_BANG_EXIT_RPM) {
                bangActive = false;
            }

            if (bangActive && !"BANG-BANG".equals(mode)) {
                bangTriggers++;
            }

            if (bangActive) {
                // Far below target: full power to spin up / recover after a shot.
                power = 1.0;
                mode = "BANG-BANG";
            } else {
                // Close to target: PIDF holds the speed smoothly.
                power = pidfPower;
                mode = "PIDF";
            }
            power = Math.max(0, Math.min(1, power));
        }
        flywheel.setPower(power);

        // Restart the "at speed" timer whenever the flywheel falls out of tolerance.
        boolean atSpeed = isAtSpeed();
        if (atSpeed && !wasAtSpeed) {
            atSpeedTimer.reset();
        }
        wasAtSpeed = atSpeed;

        measureShot();
    }

    /**
     * Shot meter (telemetry only, doesn't change control). A "shot" starts when the RPM
     * drops more than SHOT_METER_DIP_RPM below target. It records the lowest RPM, the time
     * to get back within SHOT_METER_BACK_RPM of target, and the overshoot in the
     * SHOT_METER_WATCH_SEC after that.
     */
    private void measureShot() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (bangActive) shotBangSec += dt;

        if (targetRpm <= 0) {
            shotPhase = 0;
            return;
        }
        double error = targetRpm - currentRpm;
        switch (shotPhase) {
            case 0: // waiting for a dip
                if (error > Constants.SHOT_METER_DIP_RPM) {
                    shotPhase = 1;
                    shotTimer.reset();
                    shotLowest = currentRpm;
                    shotOvershoot = 0;
                    shotBangSec = 0;
                }
                break;
            case 1: // recovering
                shotLowest = Math.min(shotLowest, currentRpm);
                if (error <= Constants.SHOT_METER_BACK_RPM) {
                    shotRecoverSec = shotTimer.seconds();
                    shotPhase = 2;
                    shotTimer.reset();
                }
                break;
            case 2: // watching for overshoot
                shotOvershoot = Math.max(shotOvershoot, currentRpm - targetRpm);
                if (shotTimer.seconds() >= Constants.SHOT_METER_WATCH_SEC) {
                    lastLowestRpm = shotLowest;
                    lastRecoverSec = shotRecoverSec;
                    lastOvershootRpm = shotOvershoot;
                    lastBangSec = shotBangSec;
                    shotsMeasured++;
                    shotPhase = 0;
                }
                break;
        }
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
    /** Which controller ran this loop: OFF, BANG-BANG or PIDF. */
    public String getMode() { return mode; }
    /** Last measured shot: lowest RPM after the dip. */
    public double getLastLowestRpm() { return lastLowestRpm; }
    /** Last measured shot: seconds from the dip until back near target. */
    public double getLastRecoverSec() { return lastRecoverSec; }
    /** Last measured shot: how far above target it went after recovering (0 = none). */
    public double getLastOvershootRpm() { return lastOvershootRpm; }
    public int getShotsMeasured() { return shotsMeasured; }
    /** Last measured shot: seconds spent at full power (bang-bang). 0 = bang-bang never ran. */
    public double getLastBangSec() { return lastBangSec; }
    /** How many times bang-bang has switched on since START. */
    public int getBangTriggers() { return bangTriggers; }
    public double getEncoderTicksPerSec() { return flywheel.getTicksPerSecond(); }
    public int getEncoderPosition() { return flywheel.getEncoderPosition(); }
    public double getBatteryVoltage() { return battery.getVoltage(); }
    public double getFlywheelCurrent() { return flywheel.getCurrent(); }
}
