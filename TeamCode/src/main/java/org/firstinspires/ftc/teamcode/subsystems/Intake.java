package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Intake motor plus the two transfer motors that push balls up to the shooter.
 *
 * Motor directions match TeleOpTest. The sign of each power constant in
 * Constants (INTAKE_POWER, TRANSFER_POWER, TRANSFER2_POWER) sets which way that
 * motor spins, and feed() uses the same signs at FEED_POWER strength.
 *
 * Creating an Intake does not move anything.
 */
public class Intake {

    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    private final DcMotorEx transfer2;

    private boolean feeding = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_NAME);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, Constants.TRANSFER_NAME);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer2 = hardwareMap.get(DcMotorEx.class, Constants.TRANSFER2_NAME);
        transfer2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Collect balls: intake + both transfer motors at full power. */
    public void intake() {
        intake.setPower(Math.signum(Constants.INTAKE_POWER));
        transfer.setPower(Math.signum(Constants.TRANSFER_POWER));
        transfer2.setPower(Math.signum(Constants.TRANSFER2_POWER));
        feeding = false;
    }

    /** Run intake + both transfer motors at FEED_POWER to push a ball into the shooter. */
    public void feed() {
        intake.setPower(Math.signum(Constants.INTAKE_POWER) * Constants.FEED_POWER);
        transfer.setPower(Math.signum(Constants.TRANSFER_POWER) * Constants.FEED_POWER);
        transfer2.setPower(Math.signum(Constants.TRANSFER2_POWER) * Constants.FEED_POWER);
        feeding = true;
    }

    public void stopFeed() {
        intake.setPower(0);
        transfer.setPower(0);
        transfer2.setPower(0);
        feeding = false;
    }

    public boolean isFeeding() {
        return feeding;
    }
}
