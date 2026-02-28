package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

        Actions.runBlocking(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-17.5))
                        .build()
        );
        Actions.runBlocking(doShotCycle(pattern, 3));



        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-37.5))
                        .build()
        );

        safeStop();
    }


    private Action doShotCycle(String pattern, int shots) {

        return new Action() {

            int shotIndex = 0;
            int attempts = 0;
            int state = 0; // 0=spinup, 1=find, 2=fireUp, 3=fireDown, 4=advance, 5=done
            long stateStart = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(true, true);
                if (!opModeIsActive()) return true;

                long now = System.currentTimeMillis();

                if (state == 0) {
                    robot.turret.setAim(true);
                    robot.outake.setPresetVelocity(Outake.FarShotVelo);
                    stateStart = now;
                    state = 1;

                }

                else if (state == 1) {

                    if (shotIndex >= shots) {
                        state = 5;
                    } else {

                        char desiredChar =
                                Character.toUpperCase(pattern.charAt(Math.min(shotIndex, 2)));

                        Spindexer.BallColor desiredColor =
                                desiredChar == 'G'
                                        ? Spindexer.BallColor.GREEN
                                        : Spindexer.BallColor.PURPLE;

                        Spindexer.BallColor visible =
                                robot.spindexer.getVisibleBallColor();

                        if (visible == desiredColor) {
                            robot.Tongue.setUp();
                            stateStart = now;
                            state = 2;
                        } else {
                            robot.spindexer.rotateByFraction(1.0 / 3.0);
                            attempts++;
                            if (attempts >= 3) {
                                shotIndex++;
                                attempts = 0;
                            }
                        }
                    }
                }

                else if (state == 2) {
                    if (now - stateStart > 200) {
                        robot.Tongue.setDown();
                        stateStart = now;
                        state = 3;
                    }
                }

                else if (state == 3) {
                    if (now - stateStart > 200) {
                        state = 4;
                    }
                }

                else if (state == 4) {
                    robot.spindexer.rotateByFraction(1.0 / 3.0);
                    shotIndex++;
                    attempts = 0;
                    state = 1;
                }

                else if (state == 5) {
                    robot.outake.intakeOff();
                    robot.turret.setAim(false);
                    return false;
                }

                return false;
            }
        };
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