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

@Autonomous(name = "Red Far", group = "Auto")
public class RedFar extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, -10, Math.toRadians(-180));
    private static final Vector2d SCAN_POSE = new Vector2d(53, -13);

    private static final long SHOT_SPINUP_MS = 1000;
    private static final long SHOT_SETTLE_MS = 250;
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
                        .turn(Math.toRadians(30))
                        .build()
        );

        pattern = detectPattern(pattern);
        doShotCycle(pattern, 3);

        safeStop();
    }

    private void doShotCycle(String pattern, int shots) {
        if (shots <= 0) {
            return;
        }

        robot.outake.setPresetVelocity(Outake.FarShotVelo);
        robot.outake.intakeOn();
        robot.Tongue.setUp();
        sleep(SHOT_SPINUP_MS);

        for (int i = 0; i < shots && opModeIsActive(); i++) {
            positionSpindexerForPattern(pattern, i);
            robot.spindexer.rotateByFraction(-1.0 / 3.0);
            waitForSpindexerIdle();
            sleep(FEED_SETTLE_MS);
        }

        sleep(SHOT_SETTLE_MS);
        robot.outake.intakeOff();
        robot.Tongue.setDown();
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
    }
}