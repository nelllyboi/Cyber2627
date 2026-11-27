package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Flywheel power sweep: measures kS and kV for FlywheelPIDF.
 *
 * Runs the flywheel at fixed powers (SWEEP_START to SWEEP_END), waits for each to
 * settle, and records the average RPM. Then it fits a straight line:
 *     power = kS + kV * RPM
 * kS is where the line starts (power to overcome friction), kV is its slope.
 *
 * Powers are converted to "12 V power" (power * battery / NOMINAL_VOLTAGE), the same
 * units Shooter uses, so the results go straight into FlywheelPIDF.
 *
 * How to use:
 *   1. Full battery. No balls loaded. Keep hands clear of the flywheel.
 *   2. INIT does not move anything. Press START to run the sweep (about 25 s).
 *   3. Press B at any time to stop the flywheel and end the sweep.
 *   4. Copy kS and kV into FlywheelPIDF.java. Check R² is close to 1 (0.99+).
 */
@Config
@TeleOp(name = "Flywheel Characterize", group = "Test")
public class FlywheelCharacterize extends LinearOpMode {

    public static double SWEEP_START = 0.2;   // first power to test
    public static double SWEEP_END   = 1.0;   // last power to test
    public static double SWEEP_STEP  = 0.1;   // power added each step
    public static double SETTLE_SEC  = 2.0;   // wait for the RPM to settle at each power
    public static double SAMPLE_SEC  = 1.0;   // then average the RPM over this long
    public static double MIN_RPM     = 100;   // ignore steps where the wheel barely turns

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Same motor setup as Shooter, so RPM readings match.
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Constants.FLYWHEEL_NAME);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FlywheelPIDF flywheel = new FlywheelPIDF(hardwareMap, motor);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Flywheel power sweep. No balls, hands clear.");
        telemetry.addLine("START = run sweep, B = stop at any time.");
        telemetry.addData("Battery (V)", battery.getVoltage());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        List<double[]> points = new ArrayList<>(); // {rpm, power at 12 V}, used for the fit
        List<String> stepLog = new ArrayList<>();  // every step, kept or not, for checking
        ElapsedTime stepTimer = new ElapsedTime();
        boolean aborted = false;

        for (double power = SWEEP_START; power <= SWEEP_END + 1e-6 && !aborted; power += SWEEP_STEP) {
            flywheel.setPower(power);
            stepTimer.reset();

            double rpmSum = 0, voltSum = 0;
            int samples = 0;

            while (opModeIsActive() && stepTimer.seconds() < SETTLE_SEC + SAMPLE_SEC) {
                if (gamepad1.b) { aborted = true; break; }

                double rpm = flywheel.getRPM();
                if (stepTimer.seconds() >= SETTLE_SEC) {
                    rpmSum += rpm;
                    voltSum += battery.getVoltage();
                    samples++;
                }

                telemetry.addData("Step power", "%.2f", power);
                telemetry.addData("Phase", stepTimer.seconds() < SETTLE_SEC ? "settling" : "sampling");
                telemetry.addData("Current RPM", rpm);
                telemetry.addData("Battery (V)", battery.getVoltage());
                telemetry.addData("Points recorded", points.size());
                telemetry.update();
            }
            if (!opModeIsActive()) aborted = true;

            if (!aborted && samples > 0) {
                double avgRpm = rpmSum / samples;
                double avgVolt = voltSum / samples;
                double power12V = power * avgVolt / Constants.NOMINAL_VOLTAGE;
                boolean kept = avgRpm >= MIN_RPM;
                if (kept) {
                    points.add(new double[]{avgRpm, power12V});
                }
                stepLog.add(String.format("  power %.2f -> %.0f RPM %s",
                        power, avgRpm, kept ? "" : "(below MIN_RPM, skipped)"));
            }
        }

        flywheel.setPower(0);

        // Least-squares line: power = kS + kV * rpm
        String result;
        double kS = 0, kV = 0, r2 = 0;
        int n = points.size();
        if (n >= 2) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (double[] p : points) {
                sx += p[0]; sy += p[1]; sxx += p[0] * p[0]; sxy += p[0] * p[1];
            }
            double denom = n * sxx - sx * sx;
            kV = (n * sxy - sx * sy) / denom;
            kS = (sy - kV * sx) / n;

            double meanY = sy / n, ssTot = 0, ssRes = 0;
            for (double[] p : points) {
                double fit = kS + kV * p[0];
                ssRes += (p[1] - fit) * (p[1] - fit);
                ssTot += (p[1] - meanY) * (p[1] - meanY);
            }
            r2 = ssTot > 0 ? 1 - ssRes / ssTot : 0;
            result = aborted ? "Stopped early (partial result)" : "Done";
        } else {
            result = aborted
                    ? "Stopped early (B or STOP) before enough steps"
                    : "Not enough points (need 2+ steps above MIN_RPM)";
        }

        while (opModeIsActive()) {
            telemetry.addData("Result", result);
            telemetry.addData("kS", "%.6f", kS);
            telemetry.addData("kV", "%.8f", kV);
            telemetry.addData("R² (1.0 = perfect line)", "%.4f", r2);
            telemetry.addData("Stopped early", aborted);
            telemetry.addLine("Every step:");
            for (String line : stepLog) {
                telemetry.addLine(line);
            }
            telemetry.addLine("Negative RPM = encoder reads backwards. 0 RPM = no encoder signal.");
            telemetry.addLine("Copy kS and kV into FlywheelPIDF.java.");
            telemetry.update();
            idle();
        }
    }
}
