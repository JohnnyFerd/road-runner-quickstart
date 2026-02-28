package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

@Autonomous(name = "Red Far Basic", group = "Auto")
public class RedFar extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, 10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53, 13);

    private static final long SHOT_SPINUP_MS = 1000;
    private static final long SHOT_SETTLE_MS = 500;
    private static final long FEED_SETTLE_MS = 2000;

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

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready for auto");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            safeStop();
            return;
        }

        // ===== Move to shooting position =====
        Actions.runBlocking(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-17.5))
                        .build()
        );

        // ===== Spin up shooter =====
        Actions.runBlocking(spinUp());
        Actions.runBlocking(waitSeconds(SHOT_SPINUP_MS / 1000.0));

        // ===== Shoot 3 balls =====
        for (int i = 0; i < 3; i++) {
            Actions.runBlocking(doOneShot());
        }

        safeStop();
    }

    /* ===================== ATOMIC ACTIONS ===================== */

    private Action spinUp() {
        return packet -> {
            robot.update(true, true);
            robot.turret.setAim(true);
            robot.outake.setPresetVelocity(Outake.FarShotVelo);
            robot.outake.intakeOn();
            return true;
        };
    }

    private Action raiseTongue() {
        return packet -> {
            robot.Tongue.setUp();
            return true;
        };
    }

    private Action lowerTongue() {
        return packet -> {
            robot.Tongue.setDown();
            return true;
        };
    }

    private Action rotateSpindexer() {
        return packet -> {
            robot.spindexer.rotateByFraction(1.0 / 3.0);
            return true;
        };
    }

    private Action waitForSpindexerIdle() {
        return packet -> {
            robot.update(true, true);
            return robot.spindexer.isIdle();
        };
    }

    private Action waitSeconds(double seconds) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true, true);
            return System.currentTimeMillis() - start >= seconds * 1000;
        };
    }

    /* ===================== COMPOSITE SHOT ===================== */

    private Action doOneShot() {
        return packet -> {
            Actions.runBlocking(waitForSpindexerIdle());
            Actions.runBlocking(raiseTongue());
            Actions.runBlocking(waitSeconds(SHOT_SETTLE_MS / 1000.0));
            Actions.runBlocking(lowerTongue());
            Actions.runBlocking(waitSeconds(FEED_SETTLE_MS / 1000.0));
            Actions.runBlocking(rotateSpindexer());
            Actions.runBlocking(waitForSpindexerIdle());
            return true;
        };
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