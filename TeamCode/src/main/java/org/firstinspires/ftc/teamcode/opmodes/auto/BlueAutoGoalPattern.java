package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Autonomous(name = "Blue Auto Goal Pattern", group = "Auto")
public class BlueAutoGoalPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(-52, -49, Math.toRadians(52.5));
    private static final Vector2d SCAN_AND_SHOOT_POSE = new Vector2d(-12, -15);
    private static final Vector2d PICKUP_1 = new Vector2d(-12, -37);
    private static final Vector2d PICKUP_2 = new Vector2d(-12, -43);
    private static final Vector2d PICKUP_3 = new Vector2d(-12, -52.5);

    private static final long SHOT_SPINUP_MS = 2000;
    private static final long SHOT_SETTLE_MS = 50;
    private static final long FEED_SETTLE_MS = 50;
    private static final long PICKUP_SETTLE_MS = 50;
    private static final long PICKUP_TIMEOUT_MS = 1400;
    private static final double HOLD_INTAKE_POWER = -0.4;

    private MecanumDrive drive;
    private Limelight3A limelight;
    private String pattern = "PPG";

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

        // ===== Move to scan/shoot pose while continuously scanning =====
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_AND_SHOOT_POSE)
                        .build(),
                continuousPatternScan()
        ));

        // ===== Shoot first 3 balls =====
        doShotCycle(pattern, 3);

        // ===== Turn for intake path =====
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(37.5)) // mirror of red side turn
                        .build()
        );

        // ===== Pick up balls =====
        collectBallAt(PICKUP_1);
        collectBallAt(PICKUP_2);
        collectBallAt(PICKUP_3);

        // ===== Shoot collected balls =====
        doShotCycle(pattern, 3);

        safeStop();
    }

    /* ===================== ATOMIC ACTIONS ===================== */
    private Action spinUpShooter() {
        return packet -> {
            robot.turret.setAim(true);
            robot.outake.setPresetVelocity(Outake.FarShotVelo);
            robot.outake.intakeOn();
            return true;
        };
    }


    private Action waitSeconds(double seconds) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true, true);
            return System.currentTimeMillis() - start >= seconds * 1000;
        };
    }

    private Action raiseTongue() {
        return packet -> { robot.Tongue.setUp(); return true; };
    }

    private Action lowerTongue() {
        return packet -> { robot.Tongue.setDown(); return true; };
    }

    private Action rotateSpindexer() {
        return packet -> {
            robot.spindexer.rotateByFraction(1.0 / 3.0);
            return true;
        };
    }
    private Action rotateSpindexerIfWrongColor(char desiredChar) {
        final Spindexer.BallColor desiredColor = desiredChar == 'G' ? Spindexer.BallColor.GREEN : Spindexer.BallColor.PURPLE;
        return packet -> {
            robot.update(true, true);

            // If the visible ball is NOT the desired color and spindexer is idle, rotate one slot
            if (robot.spindexer.isIdle() && robot.spindexer.getVisibleBallColor() != desiredColor) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
            }

            // Keep running until we see the correct color
            return robot.spindexer.getVisibleBallColor() == desiredColor;
        };
    }
    private Action waitForSpindexerIdle() {
        return packet -> { robot.update(true,true); return robot.spindexer.isIdle(); };
    }

    private Action waitForCorrectBallColor(char desiredChar) {
        final Spindexer.BallColor desiredColor = desiredChar == 'G' ? Spindexer.BallColor.GREEN : Spindexer.BallColor.PURPLE;
        return packet -> {
            robot.update(true,true);
            return robot.spindexer.getVisibleBallColor() == desiredColor;
        };
    }

    private Action holdIntakeWhileBallDetected(long timeoutMs) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.intake.setPower(HOLD_INTAKE_POWER);
            robot.update(true,true);
            return !robot.spindexer.seesBall() || (System.currentTimeMillis() - start) >= timeoutMs;
        };
    }

    private Action continuousPatternScan() {
        return packet -> {
            pattern = detectPattern(pattern);
            return false; // never ends, stops automatically when ParallelAction ends
        };
    }

    /* ===================== COMPOSITE ACTIONS ===================== */
    private void doShotCycle(String pattern, int shots) {
        if (shots <= 0 || pattern == null || pattern.length() != 3) return;

        Actions.runBlocking(spinUpShooter());
        Actions.runBlocking(waitSeconds(SHOT_SPINUP_MS / 1000.0));

        for (int i = 0; i < shots; i++) {
            char desiredChar = Character.toUpperCase(pattern.charAt(Math.min(i, 2)));
            Actions.runBlocking(rotateSpindexerIfWrongColor(desiredChar));
            Actions.runBlocking(waitForSpindexerIdle());

            Actions.runBlocking(raiseTongue());
            Actions.runBlocking(waitSeconds(FEED_SETTLE_MS / 1000.0));
            Actions.runBlocking(lowerTongue());
            Actions.runBlocking(waitForTongueDown());

            Actions.runBlocking(rotateSpindexer());
            Actions.runBlocking(waitForSpindexerIdle());
        }

        robot.outake.intakeOff();
        robot.turret.setAim(false);
    }
    private Action waitForTongueDown() {
        return packet -> {
            robot.update(true,true);
            return robot.Tongue.isDown(); // returns true when fully lowered
        };
    }
    private Action continuousIntakeAndIndex(long timeoutMs) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true, true);
            robot.intake.setPower(1);

            if (robot.spindexer.seesBall()) {
                if (robot.spindexer.isIdle()) {
                    robot.spindexer.rotateByFraction(1.0 / 3.0);
                }
            }

            return (System.currentTimeMillis() - start) >= timeoutMs;
        };
    }

    private void collectBallAt(Vector2d pickupPose) {
        robot.intake.intakeOn();
        robot.intake.setPower(1);

        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(pickupPose)
                        .build(),
                continuousIntakeAndIndex(PICKUP_TIMEOUT_MS)
        ));
    }

    /* ===================== PATTERN DETECTION ===================== */
    private String detectPattern(String fallback) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return fallback;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return fallback;

        String detected = fallback;
        for (LLResultTypes.FiducialResult tag : tags) {
            switch (tag.getFiducialId()) {
                case 21: detected = "GPP"; break;
                case 22: detected = "PGP"; break;
                case 23: detected = "PPG"; break;
            }
        }
        return detected;
    }

    /* ===================== SAFE STOP ===================== */
    private void safeStop() {
        robot.intake.intakeOff();
        robot.outake.intakeOff();
        robot.Tongue.setDown();
        limelight.stop();
        robot.update(true, true);
    }
}