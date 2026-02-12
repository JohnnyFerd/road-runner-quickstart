package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(name = "Servo Tester")
public class ServoTest extends LinearOpMode {

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    public static double power = 0.5; // power to set on the continuous servo
    boolean on = false;

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo testServo = hardwareMap.get(CRServo.class, "servo1");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Toggle servo on/off with A button
                if (currentGamepad1.a && !previousGamepad1.a) {
                    on = !on;
                }

                if (on) {
                    testServo.setPower(power);
                } else {
                    testServo.setPower(0);
                }

                telemetry.addData("Servo Active", on);
                telemetry.addData("Power", power);
                telemetry.update();
            }
        }
    }
}
