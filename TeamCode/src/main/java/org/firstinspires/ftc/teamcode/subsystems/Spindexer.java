package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer extends Subsystem {

    /* ================= Hardware ================= */

    private final CRServo servo;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    /* ================= Motion Tunables ================= */

    public static double POWER = 0.5;
    public static double TICKS_PER_REV = 8192;
    private boolean ballCurrentlyDetected = false;

    public static double kP = 0.00008;
    public static double MAX_POWER = 1;
    public static double MIN_POWER = 0.01;

    /* ================= Color Tunables ================= */

    public static double GREEN_H_MIN = 133;
    public static double GREEN_H_MAX = 170;

    public static double PURPLE_H_MIN = 171;
    public static double PURPLE_H_MAX = 220;

    public static double MIN_SATURATION = 0.35;
    public static double MIN_VALUE = 0.1;
    public static double ERROR_THRESHOLD = 5;
    public static double kD = 0.0002;
    private int lastError = 0;



    /* ================= State ================= */

    private Mode mode = Mode.IDLE;
    public static int targetTicks = 0;

    private final float[] hsv = new float[3];

    /* ================= Ball Storage ================= */

    public enum BallColor { GREEN, PURPLE }

    private final BallColor[] balls = new BallColor[3];
    private DcMotorEx motorBL;
    private int ballCount = 0;

    public enum Mode {
        IDLE,
        MOVING
    }

    public Spindexer(String servoName,
                     DcMotorEx motorBL,
                     String colorSensorName,
                     HardwareMap hwMap,
                     Telemetry telemetry) {
        this.motorBL = motorBL; // assign it
        this.telemetry = telemetry;

        servo = hwMap.get(CRServo.class, servoName);

        colorSensor = hwMap.get(ColorSensor.class, colorSensorName);


        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        if (colorSensor instanceof RevColorSensorV3) {
            ((RevColorSensorV3) colorSensor).enableLed(true);
        }
    }

    /* ================= Ball Recording ================= */

    public boolean recordBall() {

        if (ballCount >= 3) return false;

        updateHSV();

        BallColor detected = null;

        if (seesGreen()) detected = BallColor.GREEN;
        else if (seesPurple()) detected = BallColor.PURPLE;

        if (detected == null) {
            ballCurrentlyDetected = false;
            return false;
        }

        if (ballCurrentlyDetected || ballCount >= 3) {
            return false;
        }

        balls[ballCount++] = detected;
        ballCurrentlyDetected = true;
        return true;

    }

    /* ================= Shooting ================= */

    public void shoot() {
        rotateByFraction(-1.0 / 3.0);
        shiftLeft();
    }

    public void shootGreen() {
        if (ballCount > 0 && balls[0] == BallColor.GREEN) {
            shoot();
        }
    }

    public void shootPurple() {
        if (ballCount > 0 && balls[0] == BallColor.PURPLE) {
            shoot();
        }
    }

    public void catalogue() {
        rotateByFraction(1.0 / 3.0);
    }
    public int getBallCount() {
        return ballCount;
    }
    private void shiftLeft() {
        for (int i = 0; i < ballCount - 1; i++) {
            balls[i] = balls[i + 1];
        }
        if (ballCount > 0) {
            balls[ballCount - 1] = null;
            ballCount--;
        }
    }

    /* ================= Rotation Control ================= */

    public void rotateByFraction(double fraction) {

        if (fraction == 0) return;

        int delta = (int) (fraction * TICKS_PER_REV);
        targetTicks = motorBL.getCurrentPosition() + delta;

        mode = Mode.MOVING;
    }

    public void stop() {
        servo.setPower(0);
        mode = Mode.IDLE;
    }

    @Override
    public void update() {

        if (mode == Mode.MOVING) {

            int current = motorBL.getCurrentPosition();
            int error = targetTicks - current;

            if (Math.abs(error) < ERROR_THRESHOLD) {
                stop();
                lastError = 0;
                return;
            }

            int derivative = error - lastError;
            lastError = error;

            double power = (kP * error) + (kD * derivative);

            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            if (Math.abs(power) < MIN_POWER) {
                power = Math.signum(power) * MIN_POWER;
            }

            servo.setPower(power);
        }
    }

    public boolean isIdle() {
        return mode == Mode.IDLE;
    }

    /* ================= Color Detection ================= */

    private void updateHSV() {

        float r = colorSensor.red();
        float g = colorSensor.green();
        float b = colorSensor.blue();

        float max = Math.max(1f, Math.max(r, Math.max(g, b)));

        r = (r / max) * 255f;
        g = (g / max) * 255f;
        b = (b / max) * 255f;

        Color.RGBToHSV((int) r, (int) g, (int) b, hsv);
    }

    public boolean seesGreen() {
        return hsv[0] > GREEN_H_MIN &&
                hsv[0] < GREEN_H_MAX &&
                hsv[1] > MIN_SATURATION &&
                hsv[2] > MIN_VALUE;
    }

    public boolean seesPurple() {
        return hsv[0] > PURPLE_H_MIN &&
                hsv[0] < PURPLE_H_MAX &&
                hsv[1] > MIN_SATURATION &&
                hsv[2] > MIN_VALUE;
    }

    public boolean seesBall() {
        updateHSV();
        return seesGreen() || seesPurple();
    }


    /* ================= Telemetry ================= */

    @Override
    public void addTelemetry() {

        updateHSV();

        telemetry.addLine("=== Spindexer ===");

        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Ball 0", ballCount > 0 ? balls[0] : "EMPTY");
        telemetry.addData("Ball 1", ballCount > 1 ? balls[1] : "EMPTY");
        telemetry.addData("Ball 2", ballCount > 2 ? balls[2] : "EMPTY");

        telemetry.addLine("--- Color Sensor ---");
        telemetry.addData("Raw R", colorSensor.red());
        telemetry.addData("Raw G", colorSensor.green());
        telemetry.addData("Raw B", colorSensor.blue());

        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);

        telemetry.addData("Sees Green", seesGreen());
        telemetry.addData("Sees Purple", seesPurple());

        telemetry.addLine("--- Motion ---");
        telemetry.addData("Encoder", motorBL.getCurrentPosition());
        telemetry.addData("Target", targetTicks);
        telemetry.addData("Error", targetTicks - motorBL.getCurrentPosition());
        telemetry.addData("Mode", mode);
    }
}