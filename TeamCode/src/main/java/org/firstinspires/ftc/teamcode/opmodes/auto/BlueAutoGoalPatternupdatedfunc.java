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
public class BlueAutoGoalPatternupdatedfunc extends AutoBase {

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
        startPose=START_POSE;
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


}