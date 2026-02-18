package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Turret extends Subsystem {

    private JVBoysSoccerRobot robot;
    private DcMotorEx motor;

    private double startHeading;

    // gearing + encoder constants
    public double gearRatio = 85.0 / 16.0;
    public double ticksPerRev = 28 * gearRatio;
    public double ticksPerDegree = ticksPerRev / 360.0;

    // tuning
    public double power = 0.5;

    public Turret(JVBoysSoccerRobot robot) {
        this.robot = robot;

        motor = robot.hwMap.get(DcMotorEx.class, "turretspinner");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public void addTelemetry(){}

    @Override
    public void update() {

        double robotHeading = robot.getHeadingDeg();

        // turret tries to stay facing same global direction
        double targetAngle = angleWrap((startHeading + 180) - robotHeading);

        int targetTicks = (int)(targetAngle * ticksPerDegree);

        motor.setTargetPosition(targetTicks);
        motor.setPower(power);
    }

    @Override
    public void stop(){
        motor.setPower(0);
    }
}
