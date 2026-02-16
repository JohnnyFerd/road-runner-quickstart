package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Drive + RR, TurretTest")
public class TurretTest extends LinearOpMode {

    Drivetrain drivetrain;
    MecanumDrive rrDrive;
    Turret turret;


    @Override
    public void runOpMode() {

        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        turret = new Turret(robot);
        drivetrain = new Drivetrain(hardwareMap, telemetry, robot);
        double CalcRPMs, distance;
        rrDrive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0,0,0)
        );

        waitForStart();

        while(opModeIsActive()){

            // your drivetrain still drives
            drivetrain.moveXYR(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            // RoadRunner ONLY updates pose
            rrDrive.updatePoseEstimate();

            Pose2d pose = rrDrive.localizer.getPose();
            distance = Math.sqrt(Math.pow(pose.position.x, 2) + Math.pow(pose.position.y, 2));
            CalcRPMs = 14 + 1093.33333*distance;
            turret.setTargetRPM(CalcRPMs);

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("distance to goal", distance);
            telemetry.addData("Calculated rpms", CalcRPMs);
            telemetry.addData("heading", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
