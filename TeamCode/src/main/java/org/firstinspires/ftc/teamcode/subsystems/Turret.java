package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends Subsystem {

    private JVBoysSoccerRobot robot;
    private DcMotorEx RotateMotor, motor1, motor2;
    private Telemetry telemetry;

    public double gearRatio = 85.0 / 16.0;   // change later if needed
    public double motorTPR = 1700;           // RotateMotor TPR
    public double maxAngle = 170;            // wire protection
    public double wheelTPR = 28;             // REV HD Hex motor TPR

    private double targetAngle = 0;
    private double targetRPM = 0;

    public Turret(JVBoysSoccerRobot robot) {
        this.robot = robot;
        this.RotateMotor = robot.motorTURRET;
        this.motor1 = robot.motor1;
        this.motor2 = robot.motor2;
    }

    public void setTargetAngle(double angle) {
        angle = Math.max(-maxAngle, Math.min(maxAngle, angle));
        targetAngle = angle;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public double getCurrentAngle() {
        double ticks = RotateMotor.getCurrentPosition();
        double ticksPerRev = motorTPR * gearRatio;
        return (ticks / ticksPerRev) * 360.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * wheelTPR / 60.0;
    }

    @Override
    public void update() {
        // Turret control
        double current = getCurrentAngle();
        double error = targetAngle - current;

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double power = error * 0.01;   // simple P controller
        power = Math.max(-0.5, Math.min(0.5, power));
        RotateMotor.setPower(power);

        // Spin motor1 and motor2 at target RPM
        double velocity = rpmToTicksPerSecond(targetRPM);
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    @Override
    public void stop() {
        RotateMotor.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
    }

    @Override
    public void addTelemetry() {
        robot.telemetry.addData("Turret Angle", getCurrentAngle());
        robot.telemetry.addData("Turret Target", targetAngle);
        robot.telemetry.addData("Motor RPM", targetRPM);
    }
}
