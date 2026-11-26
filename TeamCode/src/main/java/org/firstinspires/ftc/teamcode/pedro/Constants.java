package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing follower config for our robot: Mecanum drivetrain + goBILDA Pinpoint localizer.
 *
 * Not the same class as org.firstinspires.ftc.teamcode.Constants.
 *
 * Every value here comes from AutoTune (see Tuning.java), not from guesses. Paste each
 * generated config in as a public static field:
 *   public static MecanumConfig drivetrainConfig = ...;   // from Mecanum Tuner
 *   public static PinpointConfig localizerConfig = ...;   // from Pinpoint Tuner
 *   public static ForesightConfig foresightConfig = ...;  // from Foresight Tuner
 *
 * Pinpoint wiring (from the Pedro docs): plug into an I2C port other than port 0,
 * sticker side up, forward pod in X, strafe pod in Y.
 */
public class Constants {
    // From Mecanum Tuner (AutoTune), 2026-11-26, after swapping the back motor wires.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // From Pinpoint Tuner (AutoTune), 2026-11-26.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.240234284889041);
        c.yPodOffset.set(1.6253196536086676);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: foresightConfig (Foresight Tuner)

    public static Follower create(HardwareMap h) {
        // Once all three configs are tuned, replace the throw with:
        // return new Follower(
        //         new PinpointLocalizer(h, localizerConfig),
        //         new Mecanum(h, drivetrainConfig),
        //         new Foresight(foresightConfig));
        throw new IllegalStateException("Pedro is not tuned yet. Run AutoTune and fill in pedro/Constants.java.");
    }
}
