package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Config
@TeleOp(name="Drive + RR, TurretTest")
public class TurretTest extends LinearOpMode {

    Drivetrain drivetrain;
    MecanumDrive rrDrive;
    Intake intake;
    Turret turret;
    Spindexer spindexer;

    String goalLabel = "";
    String patternLabel = "";

    boolean lastA = false;

    @Override
    public void runOpMode() {

        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        drivetrain = new Drivetrain(hardwareMap, telemetry, robot);
        intake = robot.intake;
        turret = robot.turret;
        spindexer = robot.spindexer;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 30);

        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        rrDrive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0,0,0)
        );

        waitForStart();

        while(opModeIsActive()) {

            robot.update(true, true);

            /* ---------------- BUTTON EDGE DETECTION ---------------- */

            boolean pressedA = gamepad1.a && !lastA;

            if (pressedA && spindexer.isIdle()) {
                spindexer.rotateByFraction(1.0/3.0);
            }

            lastA = gamepad1.a;

            /* ---------------- INTAKE CONTROLS ---------------- */

            if (gamepad1.right_bumper) intake.intakeOn();
            if (gamepad1.left_bumper) intake.intakeOff();

            /* ---------------- DRIVE ---------------- */

            drivetrain.moveXYR(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            /* ---------------- POSE UPDATE ---------------- */

            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.localizer.getPose();

            double distance = Math.hypot(pose.position.x, pose.position.y);
            double CalcRPMs = 1300 + 14.3 * distance;

            /* ---------------- LIMELIGHT ---------------- */

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (tags != null && !tags.isEmpty()) {
                    for (LLResultTypes.FiducialResult tag : tags) {

                        switch (tag.getFiducialId()) {

                            case 20:
                                goalLabel = "bluegoal";
                                break;

                            case 24:
                                goalLabel = "redgoal";
                                break;

                            case 21:
                                patternLabel = "GPP";
                                break;

                            case 22:
                                patternLabel = "PGP";
                                break;

                            case 23:
                                patternLabel = "PPG";
                                break;
                        }
                    }
                }

                telemetry.addData("Target Visible", true);
                telemetry.addData("TX", result.getTx());
                telemetry.addData("TY", result.getTy());
                telemetry.addData("TA", result.getTa());

            } else {
                telemetry.addData("Target Visible", false);
            }

            /* ---------------- TELEMETRY ---------------- */

            telemetry.addData("Goal", goalLabel);
            telemetry.addData("Pattern", patternLabel);
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Distance", distance);
            telemetry.addData("Calc RPM", CalcRPMs);
            telemetry.addData("Heading", robot.getHeadingDeg());

            telemetry.update();
        }
    }
}