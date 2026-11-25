package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * BIOBUZZ autonomous: shoot the 4 preloaded pollen from the start pose, then
 * drive to the loading zone for PARK + LEAVE.
 *
 * INIT:  gamepad1 X = BLUE, B = RED. Ball block closes so preloads can't feed early.
 *        Nothing else moves before START.
 *
 * Everything after START runs as a non-blocking state machine. Each loop:
 * update the follower and shooter, then run the current state once.
 *
 *   SPIN_UP  -> flywheel to START_SHOT_RPM, wait until it's steady at speed.
 *               Never gets there in SPIN_UP_TIMEOUT? Skip shooting, go park.
 *   FEEDING  -> open ball block, run intake + transfer. A dip in flywheel RPM
 *               means a ball went through, so stop the feed. FEED_TIMEOUT per ball.
 *   RECOVER  -> wait for the flywheel to get back to speed (RECOVER_TIMEOUT),
 *               then feed the next ball.
 *   DRIVE_TO_PARK -> close block, stop feed + flywheel, follow start -> park.
 *   DONE     -> hold position.
 *
 * Shooting ends after PRELOAD_COUNT feed attempts or SHOOT_TIME_CAP seconds.
 * At AUTO_STOP_TIME every motor stops. All numbers live in Constants.java.
 */
@Autonomous(name = "BIOBUZZ Auto", group = "Competition", preselectTeleOp = "Final TeleOp")
public class BiobuzzAuto extends LinearOpMode {

    private enum State { SPIN_UP, FEEDING, RECOVER, DRIVE_TO_PARK, DONE }

    private State state = State.SPIN_UP;

    private Shooter shooter;
    private Intake intake;
    private Follower follower;   // null if Pedro isn't tuned yet (shoot only, no park)
    private AutoPaths paths;

    private final ElapsedTime autoTimer = new ElapsedTime();   // time since START
    private final ElapsedTime stateTimer = new ElapsedTime();  // time in current state
    private final ElapsedTime shootTimer = new ElapsedTime();  // time since shooting began

    private int shotsFired = 0;     // balls confirmed by an RPM dip
    private int feedAttempts = 0;   // every feed, including ones that timed out
    private boolean stopped = false;
    private String followerError = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // The Pedro config throws until AutoTune values are pasted in. Don't crash:
        // still run the shooting part, just skip the drive.
        try {
            follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        } catch (RuntimeException e) {
            follower = null;
            followerError = e.getMessage();
        }

        // Only movement allowed in INIT: hold the preloads back.
        shooter.closeBallBlock();

        // --- INIT: alliance selection ---
        boolean blue = false;
        while (opModeInInit()) {
            if (gamepad1.x) blue = true;
            if (gamepad1.b) blue = false;

            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addLine("X = blue, B = red");
            if (follower == null) {
                telemetry.addLine("WARNING: Pedro not set up, will shoot but NOT park");
                telemetry.addData("Pedro error", followerError);
            }
            telemetry.update();
        }

        if (isStopRequested()) return;

        // --- START ---
        autoTimer.reset();
        paths = new AutoPaths(blue);
        if (follower != null) {
            follower.setPose(paths.start());
        }
        enterSpinUp();

        while (opModeIsActive()) {
            if (follower != null) {
                follower.update();
            }

            // Safety: everything stops before AUTO ends.
            if (autoTimer.seconds() >= Constants.AUTO_STOP_TIME) {
                stopEverything();
            } else {
                runStateMachine();
            }

            shooter.update();
            sendTelemetry(blue);
        }

        stopEverything();
        shooter.update();
    }

    private void runStateMachine() {
        // Leave for park once the shooting time cap is used up.
        boolean shooting = state == State.SPIN_UP || state == State.FEEDING || state == State.RECOVER;
        if (shooting && shootTimer.seconds() >= Constants.SHOOT_TIME_CAP) {
            enterDriveToPark();
            return;
        }

        switch (state) {
            case SPIN_UP:
                if (shooter.isReady()) {
                    enterFeeding();
                } else if (stateTimer.seconds() >= Constants.SPIN_UP_TIMEOUT) {
                    enterDriveToPark();   // flywheel never got to speed: skip shooting
                }
                break;

            case FEEDING:
                boolean dip = shooter.dipDetected();
                if (dip || stateTimer.seconds() >= Constants.FEED_TIMEOUT) {
                    intake.stopFeed();
                    feedAttempts++;
                    if (dip) shotsFired++;

                    if (feedAttempts >= Constants.PRELOAD_COUNT) {
                        enterDriveToPark();
                    } else {
                        setState(State.RECOVER);
                    }
                }
                break;

            case RECOVER:
                if (shooter.isReady() || stateTimer.seconds() >= Constants.RECOVER_TIMEOUT) {
                    enterFeeding();
                }
                break;

            case DRIVE_TO_PARK:
                if (follower == null || !follower.isBusy()) {
                    if (follower != null) {
                        follower.hold(paths.park());
                    }
                    setState(State.DONE);
                }
                break;

            case DONE:
                // Follower holds the park pose; nothing else to do.
                break;
        }
    }

    // --- State transitions ---

    private void setState(State next) {
        state = next;
        stateTimer.reset();
    }

    private void enterSpinUp() {
        shooter.setTargetRpm(Constants.START_SHOT_RPM);
        shootTimer.reset();
        setState(State.SPIN_UP);
    }

    private void enterFeeding() {
        shooter.openBallBlock();
        intake.feed();
        setState(State.FEEDING);
    }

    private void enterDriveToPark() {
        shooter.closeBallBlock();
        intake.stopFeed();
        shooter.stop();
        if (follower != null) {
            follower.follow(paths.toPark());
        }
        setState(State.DRIVE_TO_PARK);
    }

    private void stopEverything() {
        if (stopped) return;
        stopped = true;
        intake.stopFeed();
        shooter.stop();
        if (follower != null) {
            follower.stop();
        }
        setState(State.DONE);
    }

    private void sendTelemetry(boolean blue) {
        telemetry.addData("State", state);
        telemetry.addData("Alliance", blue ? "BLUE" : "RED");
        telemetry.addData("Time (s)", autoTimer.seconds());
        telemetry.addData("Shots fired", shotsFired);
        telemetry.addData("Feed attempts", feedAttempts);
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Current RPM", shooter.getCurrentRpm());
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Shooter power", shooter.getPower());
        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("X (in)", pose.x());
            telemetry.addData("Y (in)", pose.y());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
            telemetry.addData("Follower busy", follower.isBusy());
        } else {
            telemetry.addLine("Pedro not set up: no drive, no pose");
        }
        if (stopped) {
            telemetry.addLine("AUTO_STOP_TIME reached: all motors stopped");
        }
        telemetry.update();
    }
}
