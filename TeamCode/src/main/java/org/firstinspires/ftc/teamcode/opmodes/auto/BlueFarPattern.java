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

@Autonomous(name = "Blue Far Pattern", group = "Auto")
public class BlueFarPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, -10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53, -13);

    private static final long SHOT_SPINUP_MS = 2000;
    private static final long SHOT_SETTLE_MS = 50;
    private static final long FEED_SETTLE_MS = 50;

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

        // ===== Move to scan pose while scanning pattern =====
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(360+17.5))
                        .build(),
                continuousPatternScan()
        ));

        // ===== Shoot first 3 balls =====
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
    private Action waitSeconds(double seconds) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true,true);
            return System.currentTimeMillis() - start >= seconds * 1000;
        };
    }

    private Action continuousPatternScan() {
        return packet -> {
            pattern = detectPattern(pattern);
            return false;
        };
    }

    /* ===================== COMPOSITE ACTIONS ===================== */
    private void doShotCycle(String pattern, int shots) {

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
        robot.Tongue.setDown();
        robot.turret.setAim(false);
    }

    /* ===================== PATTERN DETECTION ===================== */
    private Action waitForTongueDown() {
        return packet -> {
            robot.update(true,true);
            return robot.Tongue.isDown(); // returns true when fully lowered
        };
    }
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
        robot.update(true,true);
    }
}