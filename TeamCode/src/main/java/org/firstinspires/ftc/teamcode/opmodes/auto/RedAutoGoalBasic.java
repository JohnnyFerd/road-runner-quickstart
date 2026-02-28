package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;

@Autonomous(name = "Red Auto Goal Basic", group = "Auto")
public class RedAutoGoalBasic extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(-52, 49, Math.toRadians(127.5));
    private static final Vector2d SCAN_AND_SHOOT_POSE = new Vector2d(-12, 15);
    private static final Vector2d PICKUP_1 = new Vector2d(-12, 37);
    private static final Vector2d PICKUP_2 = new Vector2d(-12, 43);
    private static final Vector2d PICKUP_3 = new Vector2d(-12, 52.5);

    private static final long SHOT_SPINUP_MS = 2000;
    private static final long FEED_SETTLE_MS = 50;
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

            telemetry.addLine("Ready for auto");
            telemetry.addData("Detected pattern", pattern);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            safeStop();
            return;
        }

        // Move to shoot pose
        Actions.runBlocking(drive.actionBuilder(START_POSE)
                .strafeTo(SCAN_AND_SHOOT_POSE)
                .build());



        // Shoot preloaded balls
        doShotCycle(3);

        // Turn toward pickup path
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .turn(Math.toRadians(-37.5))
                .build());

        // Collect balls
        collectAtPose(PICKUP_1);
        collectAtPose(PICKUP_2);
        collectAtPose(PICKUP_3);

        // Shoot collected balls
        doShotCycle(3);

        safeStop();
    }

    /* ===================== ROADRUNNER ATOMIC ACTIONS ===================== */
    private Action raiseTongue() {
        return packet -> { robot.Tongue.setUp(); return true; };
    }

    private Action lowerTongue() {
        return packet -> { robot.Tongue.setDown(); return true; };
    }

    private Action rotateSpindexer() {
        return packet -> { robot.spindexer.rotateByFraction(1.0 / 3.0); return true; };
    }

    private Action waitForSpindexerIdle() {
        return packet -> { robot.update(true,true); return robot.spindexer.isIdle(); };
    }

    private Action waitSeconds(double seconds) {
        final long start = System.currentTimeMillis();
        return packet -> { robot.update(true,true); return System.currentTimeMillis() - start >= seconds * 1000; };
    }

    /* ===================== SHOOTING CYCLE ===================== */
    private void doShotCycle(int shots) {
        if (shots <= 0) return;

        robot.turret.setAim(true);
        robot.outake.setPresetVelocity(Outake.FarShotVelo);
        robot.outake.intakeOn();
        robot.Tongue.setDown();

        Actions.runBlocking(waitSeconds(0.2)); // shooter spinup

        for (int i = 0; i < shots; i++) {
            Actions.runBlocking(raiseTongue());
            Actions.runBlocking(waitSeconds(FEED_SETTLE_MS / 1000.0));
            Actions.runBlocking(lowerTongue());
            Actions.runBlocking(waitSeconds(FEED_SETTLE_MS / 1000.0));
            Actions.runBlocking(rotateSpindexer());
            Actions.runBlocking(waitForSpindexerIdle());
        }

        robot.outake.intakeOff();
        robot.Tongue.setDown();
        robot.turret.setAim(false);
    }

    /* ===================== BALL PICKUP ===================== */
    private void collectAtPose(Vector2d pickupPose) {
        robot.intake.intakeOn();
        robot.intake.setPower(1);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(pickupPose)
                .build());

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < PICKUP_TIMEOUT_MS) {
            robot.update(true,true);
            if (robot.spindexer.recordBall()) {
                if (robot.spindexer.isIdle()) {
                    robot.spindexer.rotateByFraction(1.0 / 3.0);
                }
                sleep(30);
                break;
            }
            sleep(10);
        }

        while (opModeIsActive() && !robot.spindexer.isIdle()) {
            robot.update(true,true);
            sleep(10);
        }
    }

    /* ===================== PATTERN DETECTION ===================== */
    private String detectPattern(String fallback) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return fallback;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return fallback;

        String detected = fallback;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == 21) detected = "GPP";
            else if (tag.getFiducialId() == 22) detected = "PGP";
            else if (tag.getFiducialId() == 23) detected = "PPG";
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