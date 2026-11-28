package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Mecanum drivetrain for TeleOp.
 *
 * Inputs are in the range -1..1:
 *   forward: + drives forward
 *   strafe:  + drives right
 *   turn:    + turns clockwise (right)
 *
 * Wheel powers are scaled down together if any would go over 1.0, so the robot
 * still moves in the direction you asked for, just a bit slower.
 *
 * Creating a Drivetrain does not move anything.
 */
public class Drivetrain {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    // Last powers sent, for telemetry.
    private double flPower, frPower, blPower, brPower;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft  = setUpMotor(hardwareMap, Constants.FRONT_LEFT_NAME,  Constants.FRONT_LEFT_REVERSED);
        frontRight = setUpMotor(hardwareMap, Constants.FRONT_RIGHT_NAME, Constants.FRONT_RIGHT_REVERSED);
        backLeft   = setUpMotor(hardwareMap, Constants.BACK_LEFT_NAME,   Constants.BACK_LEFT_REVERSED);
        backRight  = setUpMotor(hardwareMap, Constants.BACK_RIGHT_NAME,  Constants.BACK_RIGHT_REVERSED);
    }

    private static DcMotorEx setUpMotor(HardwareMap hardwareMap, String name, boolean reversed) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    /** Robot-centric drive: forward means the way the robot is facing. */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Keep the ratio between wheels, but never ask a motor for more than 1.0.
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        setPowers(fl, fr, bl, br);
    }

    /**
     * Field-centric drive: forward means "away from the driver" no matter which
     * way the robot faces. headingRad is the robot heading (counter-clockwise positive)
     * measured from the direction it faced when the heading was zeroed.
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Rotate the stick direction by -heading to get it in robot coordinates.
        double robotStrafe  = strafe * cos + forward * sin;
        double robotForward = -strafe * sin + forward * cos;

        drive(robotForward, robotStrafe, turn);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    private void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
        flPower = fl;
        frPower = fr;
        blPower = bl;
        brPower = br;
    }

    public double getFrontLeftPower()  { return flPower; }
    public double getFrontRightPower() { return frPower; }
    public double getBackLeftPower()   { return blPower; }
    public double getBackRightPower()  { return brPower; }

    // Motor current draw in amps (each read takes a few ms, so only call when shown).
    public double getFrontLeftCurrent()  { return frontLeft.getCurrent(CurrentUnit.AMPS); }
    public double getFrontRightCurrent() { return frontRight.getCurrent(CurrentUnit.AMPS); }
    public double getBackLeftCurrent()   { return backLeft.getCurrent(CurrentUnit.AMPS); }
    public double getBackRightCurrent()  { return backRight.getCurrent(CurrentUnit.AMPS); }
}
