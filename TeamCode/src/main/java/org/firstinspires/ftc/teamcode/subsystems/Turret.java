package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Turret extends Subsystem {

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

    // tuning
    public static double power = 0.5;

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
    public void addTelemetry(){


    }

    @Override
    public void update() {

        robotHeading = robot.getHeadingDeg();

        // turret tries to stay facing same global direction
        double delta = angleWrap(robot.getHeadingDeg() - startHeading);
        int ticks = (int)(-delta * gearRatio*ticksperdeg); // temp scale

        motor.setTargetPosition(ticks);
        motor.setPower(0.5);



    }

    @Override
    public void stop(){
        motor.setPower(0);
    }
}
