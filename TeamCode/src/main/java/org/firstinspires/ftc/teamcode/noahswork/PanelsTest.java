package org.firstinspires.ftc.teamcode.noahswork;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Checks that the Panels dashboard works. No hardware needed, nothing moves.
 *
 * 1. Connect a laptop to the robot's WiFi and open http://192.168.43.1:8001
 * 2. Run this OpMode.
 * 3. Telemetry: "Loop count" should keep going up.
 * 4. Graph: pick "wave" and you should see a sine wave.
 * 5. Configurables: find PanelsTest and change WAVE_SPEED. The wave should speed up or slow down.
 */
@Configurable
@TeleOp(name = "Panels Test", group = "Test")
public class PanelsTest extends LinearOpMode {

    // Shows up in Panels' Configurables tab. Must be public static (and not final) to be editable.
    public static double WAVE_SPEED = 1.0; // wave cycles per second

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void runOpMode() {
        panels.addLine("Press START, then open the Graph widget in Panels");
        panels.update(telemetry);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int loopCount = 0;
        double phase = 0;  // keeps the wave smooth when WAVE_SPEED changes
        double lastTime = 0;

        while (opModeIsActive()) {
            double now = timer.seconds();
            phase += 2 * Math.PI * WAVE_SPEED * (now - lastTime);
            lastTime = now;
            loopCount++;

            panels.addData("Loop count", loopCount);
            panels.addData("WAVE_SPEED", WAVE_SPEED);
            panels.addData("wave", Math.sin(phase)); // graphed value

            panels.update(telemetry); // updates Panels and the Driver Station
        }
    }
}
