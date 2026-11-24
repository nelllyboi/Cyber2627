package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public static Follower create(HardwareMap h) {
        // Once all three configs are tuned, replace the throw with:
        // return new Follower(
        //         new PinpointLocalizer(h, localizerConfig),
        //         new Mecanum(h, drivetrainConfig),
        //         new Foresight(foresightConfig));
        throw new IllegalStateException("Pedro is not tuned yet. Run AutoTune and fill in pedro/Constants.java.");
    }
}
