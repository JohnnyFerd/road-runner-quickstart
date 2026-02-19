package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer extends Subsystem {

    /* ===== Hardware ===== */
    private final CRServo servo;
    private final DcMotorEx encoderMotor;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    /* ===== Motion Tunables ===== */
    public static double POWER = 0.5;
    public static double TICKS_PER_REV =  8192; // change to your encoder

    /* ===== HSV Tunables ===== */
    public static double GREEN_H_MIN = 133;
    public static double GREEN_H_MAX = 170;
    public static double PURPLE_H_MIN = 171;
    public static double PURPLE_H_MAX = 220;

    /* ===== State ===== */
    private Mode mode = Mode.IDLE;
    public static int targetTicks = 0;

    private final float[] hsv = new float[3];

    /* ===== Ball Storage ===== */
    public enum BallColor { GREEN, PURPLE }
    private final BallColor[] balls = new BallColor[3];
    private int ballCount = 0;

    public enum Mode {
        IDLE,
        MOVING
    }

    public Spindexer(String servoName, String encoderName, String colorSensorName,
                     HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;
        servo = hwMap.get(CRServo.class, servoName);
        encoderMotor = hwMap.get(DcMotorEx.class, encoderName);
        colorSensor = hwMap.get(ColorSensor.class, colorSensorName);
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* ===== Intake Record ===== */
    public void recordBall() {
        if (ballCount >= 3) return;
        updateHSV();
        if (seesGreen()) balls[ballCount++] = BallColor.GREEN;
        else if (seesPurple()) balls[ballCount++] = BallColor.PURPLE;
    }

public void setModeMoving(){ mode = Mode.MOVING;}
    /* ===== Shoot Logic ===== */
    public void shoot() {
        rotateByFraction(-1.0/3.0);
        if (ballCount > 0) shiftLeft();
    }

    public void shootGreen() {
        if (ballCount > 0 && balls[0] == BallColor.GREEN) shoot();
    }

    public void shootPurple() {
        if (ballCount > 0 && balls[0] == BallColor.PURPLE) shoot();
    }

    public void catalogue() {
        rotateByFraction(1.0/3.0);
    }

    /* ===== Rotation Control ===== */
    public void rotateByFraction(double fraction) {
        if (fraction == 0) return;

        int delta = (int)(fraction * TICKS_PER_REV);
        targetTicks = encoderMotor.getCurrentPosition() + delta;

        mode = Mode.MOVING;

        telemetry.addData("Rotate Fraction", fraction);
        telemetry.addData("Target", targetTicks);
        telemetry.update();
    }

    public void stop() {
        servo.setPower(0);
        mode = Mode.IDLE;
    }

    private void shiftLeft() {
        for (int i = 0; i < ballCount - 1; i++)
            balls[i] = balls[i+1];
        balls[--ballCount] = null;
    }

    /* ===== Color Detection ===== */
    private void updateHSV() {
        int r = Math.min(colorSensor.red(),255);
        int g = Math.min(colorSensor.green(),255);
        int b = Math.min(colorSensor.blue(),255);
        Color.RGBToHSV(r,g,b,hsv);
    }

    private boolean seesGreen() {
        return hsv[0] > GREEN_H_MIN && hsv[0] < GREEN_H_MAX;
    }

    private boolean seesPurple() {
        return hsv[0] > PURPLE_H_MIN && hsv[0] < PURPLE_H_MAX;
    }

    public static double kP = 0.0008;   // tune this
    public static double MAX_POWER = 0.6;
    public static double MIN_POWER = 0.08;  // prevents stall

    @Override
    public void update() {

        if (mode == Mode.MOVING) {

            int current = encoderMotor.getCurrentPosition();
            int error = targetTicks - current;

            if (Math.abs(error) < 30) {   // tighter threshold
                stop();
                return;
            }

            double power = kP * error;

            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            if (Math.abs(power) < MIN_POWER)
                power = Math.signum(power) * MIN_POWER;

            servo.setPower(power);
        }
    }

    public boolean isIdle() {
        return mode == Mode.IDLE;
    }

    /* ===== Telemetry ===== */
    @Override
    public void addTelemetry() {
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Ball 0", ballCount>0 ? balls[0] : "EMPTY");
        telemetry.addData("Ball 1", ballCount>1 ? balls[1] : "EMPTY");
        telemetry.addData("Ball 2", ballCount>2 ? balls[2] : "EMPTY");
        telemetry.addData("Encoder", encoderMotor.getCurrentPosition());
        telemetry.addData("Target", targetTicks);
        telemetry.addData("Mode", mode);
    }
}
