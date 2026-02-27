package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;

@Autonomous(name = "Red Auto Goal", group = "Auto")
public class RedAutoGoal extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(-52, 49, Math.toRadians(127.5));
    private static final Vector2d SCAN_AND_SHOOT_POSE = new Vector2d(-12, 15);
    private static final Vector2d PICKUP_1 = new Vector2d(-12, 37);
    private static final Vector2d PICKUP_2 = new Vector2d(-12, 43);
    private static final Vector2d PICKUP_3 = new Vector2d(-12, 52.5);

    private static final long SHOT_SPINUP_MS = 2000;
    private static final long SHOT_SETTLE_MS = 50;
    private static final long FEED_SETTLE_MS = 50;
    private static final long PICKUP_SETTLE_MS = 50;
    private static final long PICKUP_TIMEOUT_MS = 1400;

    private static final double HOLD_INTAKE_POWER = -0.4;

    private MecanumDrive drive;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        initialize();
        drive = new MecanumDrive(hardwareMap, START_POSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        robot.Tongue.setDown();
        robot.outake.intakeOff();
        robot.intake.intakeOff();

        String pattern = "PPG";

        while (!isStarted() && !isStopRequested()) {
            pattern = detectPattern(pattern);
            telemetry.addLine("Ready for auto");
            telemetry.addData("Detected pattern", pattern);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            safeStop();
            return;
        }

        Actions.runBlocking(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_AND_SHOOT_POSE)
                        .build()
        );

        pattern = detectPattern(pattern);

        doShotCycle(pattern, 3);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-37.5))
                        .build()
        );
        robot.intake.intakeOn();
        collectAtPose(PICKUP_1);
        robot.intake.intakeOn();
        collectAtPose(PICKUP_2);
        robot.intake.intakeOn();
        collectAtPose(PICKUP_3);

        doShotCycle(pattern, robot.spindexer.getBallCount());

        safeStop();
    }

    private void collectAtPose(Vector2d pickupPose) {
        robot.intake.intakeOn();
        robot.intake.setPower(1);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(pickupPose)
                        .build()
        );

        // If a ball is already at the color sensor, keep intake at -0.4 while indexing it away.
        if (robot.spindexer.seesBall()) {
            if (robot.spindexer.isIdle()) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
            }
            holdIntakeWhileBallDetected(PICKUP_TIMEOUT_MS);
        }

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < PICKUP_TIMEOUT_MS) {
            robot.update(true, true);
            if (robot.spindexer.recordBall()) {
                if (robot.spindexer.isIdle()) {
                    robot.spindexer.rotateByFraction(1.0 / 3.0);
                }
                holdIntakeWhileBallDetected(PICKUP_SETTLE_MS + 400);
                sleep(PICKUP_SETTLE_MS);
                break;
            }
            sleep(30);
        }



        while (opModeIsActive() && !robot.spindexer.isIdle()) {
            robot.update(true, true);
            sleep(10);
        }
    }

    private void waitWithUpdates(long delayMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < delayMs) {
            robot.update(true, true);
            sleep(10);
        }
    }
    private void holdIntakeWhileBallDetected(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && robot.spindexer.seesBall()
                && (System.currentTimeMillis() - start) < timeoutMs) {
            robot.intake.setPower(HOLD_INTAKE_POWER);
            robot.update(true, true);
            sleep(10);
        }
    }

    private void doShotCycle(String pattern, int shots) {
        if (shots <= 0) {
            return;
        }
        robot.turret.setAim(true);

        robot.outake.setPresetVelocity(Outake.FarShotVelo);
        sleep(200);
        robot.outake.intakeOn();
        robot.Tongue.setDown();

        while ( robot.Tongue.isBusy()) {
            robot.update(true,true);   // or whatever runs subsystem updates
        }
        for (int i = 0; i < shots && opModeIsActive(); i++) {

            robot.Tongue.setDown();
            waitWithUpdates(FEED_SETTLE_MS);

            positionSpindexerForPattern(pattern, i);
            robot.spindexer.rotateByFraction(-1.0 / 3.0);
            waitForSpindexerIdle();
            robot.Tongue.setUp();
            waitWithUpdates(FEED_SETTLE_MS);

        }


        robot.outake.intakeOff();
        robot.Tongue.setDown();
        robot.turret.setAim(false);

    }

    private void waitForSpindexerIdle() {
        while (opModeIsActive() && !robot.spindexer.isIdle()) {
            robot.update(true, true);
            sleep(10);
        }
    }

    private void alignSpindexerForDesiredColor(String pattern, int shotIndex) {
        if (pattern == null || pattern.length() != 3) {
            return;
        }

        int desiredIndex = Math.min(shotIndex, 2);

        if (robot.spindexer.isIdle()) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
                char desiredColor = Character.toUpperCase(pattern.charAt(desiredIndex));
                if (desiredColor != 'G' && desiredColor != 'P') {
                    return;
                }

                int attempts = 0;
                while (opModeIsActive() && attempts < 3) {
                    boolean ballVisible = robot.spindexer.seesBall();
                    boolean matchesDesired = (desiredColor == 'G' && ballVisible && robot.spindexer.seesGreen())
                            || (desiredColor == 'P' && ballVisible && robot.spindexer.seesPurple());

                    if (matchesDesired) {
                        return;
                    }

                    if (!robot.spindexer.isIdle()) {

                        continue;
                    }

                    robot.spindexer.rotateByFraction(1.0 / 3.0);
                    waitForSpindexerIdle();
                    sleep(40);
                    attempts++;
                }

        }
    }

    private void positionSpindexerForPattern(String pattern, int shotIndex) {
        if (pattern == null || pattern.length() != 3) {
            return;
        }

        int desiredIndex = Math.min(shotIndex, 2);
        if (pattern.charAt(desiredIndex) == 'G') {
            if (robot.spindexer.isIdle()) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
                waitForSpindexerIdle();
            }
        }
    }

    private String detectPattern(String fallback) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return fallback;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return fallback;
        }

        String detected = fallback;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == 21) {
                detected = "GPP";
            } else if (tag.getFiducialId() == 22) {
                detected = "PGP";
            } else if (tag.getFiducialId() == 23) {
                detected = "PPG";
            }
        }
        return detected;
    }


    private void safeStop() {
        robot.intake.intakeOff();
        robot.outake.intakeOff();
        robot.Tongue.setDown();
        limelight.stop();
        robot.update(true, true);
    }
}
