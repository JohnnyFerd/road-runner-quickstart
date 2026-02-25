package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "PlanA_RED ")
public class PlanA_RED  extends AutoBase {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive with 180° heading (turned around)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        waitForStart();
        if (isStopRequested()) return;

        // Move Back Before First Shot
        Pose2d current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeTo(new Vector2d(current.position.x + 40, current.position.y))
                        .build()
        );
       current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .turnTo(Math.toRadians( 30))
                        .build()
        );
//shoo

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)

                        .strafeTo(new Vector2d(current.position.x - 0, current.position.y + 40 ))
                        .build()
        );



    }
}