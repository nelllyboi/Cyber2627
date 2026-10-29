package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // --- Hardware config names (final, not shown in Dashboard) ---
    public static final String FLYWHEEL_NAME   = "shooter";
    public static final String TRANSFER_NAME   = "transferMotor";
    public static final String TRANSFER2_NAME  = "transferMotor2";
    public static final String INTAKE_NAME     = "intakeMotor";
    public static final String BALL_BLOCK_NAME = "ballBlock";

    // --- Ball block servo ---
    public static double BLOCK_MIN = 0.85;   // open position
    public static double BLOCK_MAX = 0.6;   // blocking position

    // --- Flywheel ---
    public static double RPM_STEP  = 100;   // RPM change per dpad press
    public static double START_RPM = 1000;  // initial target RPM

    // --- Motor powers ---
    public static double INTAKE_POWER    = -1.0;
    public static double TRANSFER_POWER  = 1.0;
    public static double TRANSFER2_POWER = 1.0;  // flip sign if it spins the wrong way
}