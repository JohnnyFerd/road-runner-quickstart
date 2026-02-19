package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Config
@TeleOp(name="Drive + RR, TurretTest")
public class TurretTest extends LinearOpMode {

    Drivetrain drivetrain;
    MecanumDrive rrDrive;
    Intake intake;
    Turret turret;
Spindexer spindexer;

    @Override
    public void runOpMode() {

        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        drivetrain = new Drivetrain(hardwareMap, telemetry, robot);
        intake=robot.intake;
        turret=robot.turret;
        spindexer=robot.spindexer;

        double CalcRPMs, distance;
        rrDrive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0,0,0)
        );

        waitForStart();

        while(opModeIsActive()){
            robot.update(true, true);
            if (gamepad1.y && spindexer.isIdle()) {
                spindexer.rotateByFraction(1.0 / 3.0);
            }

            if(gamepad1.a){
                intake.intakeOn();
            }
            if(gamepad1.b){
                intake.intakeOff();
            }
            // your drivetrain still drives
            drivetrain.moveXYR(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );



            // RoadRunner ONLY updates pose
            rrDrive.updatePoseEstimate();
            Pose2d pose = rrDrive.localizer.getPose();
            distance = pose.position.x;
            CalcRPMs = 1300 + 14.3*distance;


            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("distance to goal", distance);
            telemetry.addData("Calculated rpms", CalcRPMs);
            telemetry.addData("heading", robot.getHeadingDeg());


            telemetry.update();
        }
    }
}
