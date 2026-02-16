package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


@Config
@TeleOp(name = "Limelight 3A Test")
public class Limelight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Use pipeline 0 (change if needed)
        limelight.pipelineSwitch(0);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 30);
        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addData("Target Visible", true);
                telemetry.addData("TX (Horizontal Offset)", result.getTx());
                telemetry.addData("TY (Vertical Offset)", result.getTy());
                telemetry.addData("TA (Target Area)", result.getTa());
                telemetry.addData("bot pose", result.getBotpose());


            } else {
                telemetry.addData("Target Visible", false);
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
