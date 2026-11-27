package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing AutoTune procedures. Run them from http://192.168.43.1:10158
 * while connected to the Robot Controller.
 *
 * Tuning order (https://pedropathing.com/docs/pathing/tuning):
 *   1. Mecanum Tuner  -> paste the generated MecanumConfig into Constants.drivetrainConfig
 *   2. Pinpoint Tuner -> paste the generated PinpointConfig into Constants.localizerConfig
 *   3. Uncomment foresightTuner() and tests() below, then run Foresight Tuner
 *      -> paste the generated ForesightConfig into Constants.foresightConfig
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // Enabled now that drivetrainConfig and localizerConfig exist in Constants.
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                null); // replace null with () -> new Foresight(Constants.foresightConfig) once tuned
    }
}
