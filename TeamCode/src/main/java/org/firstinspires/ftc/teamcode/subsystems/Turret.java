package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Turret extends Subsystem {

    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private DcMotorEx motor;

    private double startHeading;

    // runtime debug values
    private double robotHeading = 0;
    private double targetAngle= 0;
    private int targetTicks= 0;

    // gearing + encoder constants
    public static double gearRatio = 85.0 / 16.0;
    public static double ticksperdeg = 560.0/360;
    public static double MAX_TURRET_DEG = 60.0;

    // tuning

    public static double kA = 0.05;

    public Turret(JVBoysSoccerRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        motor = robot.hwMap.get(DcMotorEx.class, "turretspinner");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startHeading = robot.getHeadingDeg();
    }

    private double angleWrap(double angle){
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;
        return angle;
    }

    @Override
    public void addTelemetry(){


    }


    @Override
    public void update() {

        robotHeading = robot.getHeadingDeg();

        // how much robot rotated since start
        double delta = angleWrap(robotHeading - startHeading);

        // CLAMP turret allowed rotation to ±60°
        double clampedDelta = Math.max(-MAX_TURRET_DEG,
                Math.min(MAX_TURRET_DEG, delta));

        // detect if we tried to exceed range
        boolean outOfRange = Math.abs(delta) > MAX_TURRET_DEG;

        // convert to encoder ticks
        targetAngle = clampedDelta;
        targetTicks = (int)(-clampedDelta * gearRatio * ticksperdeg);

        motor.setTargetPosition(targetTicks);

        // basic proportional power
        double power = Math.abs(clampedDelta * kA);
        power = Math.max(power, 0.1);

        motor.setPower(power);

        telemetry.addLine("=== Turret ===");
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Start Heading", startHeading);
        telemetry.addData("Delta", delta);
        telemetry.addData("Clamped Delta", clampedDelta);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Out Of Range", Math.abs(delta) > MAX_TURRET_DEG);
    }

    @Override
    public void stop(){
        motor.setPower(0);
    }
}
