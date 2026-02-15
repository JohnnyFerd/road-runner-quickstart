package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret extends Subsystem {

    private JVBoysSoccerRobot robot;
    private DcMotorEx motor;

    // ====== TUNABLE ======
    public double gearRatio = 85.0 / 16.0;   // change later
    public double motorTPR = 1700;          // change for your motor
    public double maxAngle = 170;            // wire protection
    // =====================

    private double targetAngle = 0;

    public Turret(JVBoysSoccerRobot robot) {
        this.robot = robot;
        this.motor = robot.motorTURRET;
    }

    public void setTargetAngle(double angle) {

        // Wire protection clamp
        angle = Math.max(-maxAngle, Math.min(maxAngle, angle));

        targetAngle = angle;
    }

    public double getCurrentAngle() {
        double ticks = motor.getCurrentPosition();
        double ticksPerRev = motorTPR * gearRatio;
        return (ticks / ticksPerRev) * 360.0;
    }

    @Override
    public void update() {

        double current = getCurrentAngle();
        double error = targetAngle - current;

        // Shortest path logic
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double power = error * 0.01;   // simple P controller
        power = Math.max(-0.5, Math.min(0.5, power));

        motor.setPower(power);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void addTelemetry() {
        robot.telemetry.addData("Turret Angle", getCurrentAngle());
        robot.telemetry.addData("Turret Target", targetAngle);
    }
}
