package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tongue Servo Tester", group = "Test")
public class TongueServoTester extends LinearOpMode {

    private Servo tongue1;
    private Servo tongue2;

    private boolean flipped = false;
    private boolean lastA = false;

    public static double angle = 0;

    @Override
    public void runOpMode() {

        tongue1 = hardwareMap.get(Servo.class, "Tongue1");
        tongue2 = hardwareMap.get(Servo.class, "Tongue2");

        // Initial positions
        tongue1.setPosition(0);
        tongue2.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            tongue1.setPosition(angle);
            tongue2.setPosition();

            telemetry.update();
        }
    }
}
