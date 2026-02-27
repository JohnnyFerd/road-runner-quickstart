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
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Autonomous(name = "Red Far Pattern", group = "Auto")
public class RedFarPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60,  10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53,  13);

    private static final long SHOT_SPINUP_MS = 1000;
    private static final long SHOT_SETTLE_MS = 500;
    private static final long FEED_SETTLE_MS = 500;

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
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-17.5))
                        .build()
        );

        pattern = detectPattern(pattern);
        doShotCycle(pattern, 3);

        safeStop();
    }

    private void doShotCycle(String pattern, int shots) {
        if (shots <= 0 || pattern == null || pattern.length() != 3) return;

        robot.turret.setAim(true);
        robot.outake.setPresetVelocity(Outake.FarShotVelo);

        long spinStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - spinStart < SHOT_SPINUP_MS) {
            robot.update(true, true);
        }

        for (int i = 0; i < shots && opModeIsActive(); i++) {

            // Determine desired color for this shot
            char desiredChar = Character.toUpperCase(pattern.charAt(Math.min(i, 2)));
            Spindexer.BallColor desiredColor =
                    desiredChar == 'G'
                            ? Spindexer.BallColor.GREEN
                            : Spindexer.BallColor.PURPLE;

            // Rotate up to 3 times to find correct color
            int attempts = 0;
            while (opModeIsActive() && attempts < 3) {

                waitForSpindexerIdle();
                Spindexer.BallColor visible = robot.spindexer.getVisibleBallColor();

                if (visible == desiredColor) {
                    break;
                }

                robot.spindexer.rotateByFraction(1.0 / 3.0);
                waitForSpindexerIdle();
                attempts++;
            }

            // Fire
            robot.Tongue.setUp();
            waitWithUpdates(FEED_SETTLE_MS);
            robot.Tongue.setDown();
            waitWithUpdates(SHOT_SETTLE_MS);

            // Advance to next chamber
            robot.spindexer.rotateByFraction(1.0 / 3.0);
            waitForSpindexerIdle();
        }

        robot.outake.intakeOff();
        robot.turret.setAim(false);
    }

    private void waitWithUpdates(long delayMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < delayMs) {
            robot.update(true, true);
            sleep(10);
        }
    }

    private void positionSpindexerForPattern(String pattern, int shotIndex) {
        if (pattern == null || pattern.length() != 3) {
            return;
        }

        int desiredIndex = Math.min(shotIndex, 2);
        if (pattern.charAt(desiredIndex) == 'G' && robot.spindexer.isIdle()) {
            robot.spindexer.rotateByFraction(1.0 / 3.0);
            waitForSpindexerIdle();
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

    private void waitForSpindexerIdle() {
        while (opModeIsActive() && !robot.spindexer.isIdle()) {
            robot.update(true, true);
            sleep(10);
        }
    }

    private void safeStop() {
        robot.intake.intakeOff();
        robot.outake.intakeOff();
        robot.Tongue.setDown();
        limelight.stop();
        robot.update(true, true);
        waitWithUpdates(200);
    }
}