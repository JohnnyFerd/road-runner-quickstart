package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Tongue extends Subsystem {

    private final Servo tongue1;
    private final Servo tongue2;
    private final Telemetry telemetry;

    /* ===== Tunables ===== */
    public static double DOWN_POSITION = 0.03;
    public static double UP_POSITION = 0.34;
    public static double MOVE_TIME_MS = 300;   // time for servo to physically move

    /* ===== State ===== */
    private Mode targetMode = Mode.DOWN;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private boolean moving = false;

    private enum Mode {
        UP,
        DOWN
    }

    public Tongue(String tongue1Name, String tongue2Name,
                  HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        tongue1 = hwMap.get(Servo.class, tongue1Name);
        tongue2 = hwMap.get(Servo.class, tongue2Name);

        setDown();  // initialize
    }

    /* ===== Commands ===== */

    public void setUp() {
        targetMode = Mode.UP;
        moving = true;
        moveTimer.reset();
    }

    public void setDown() {
        targetMode = Mode.DOWN;
        moving = true;
        moveTimer.reset();
    }

    public boolean isBusy() {
        return moving;
    }

    public boolean isUp() {
        return targetMode == Mode.UP && !moving;
    }

    /* ===== Update Loop ===== */

    @Override
    public void update() {

        // Apply position continuously
        switch (targetMode) {
            case UP:
                tongue1.setPosition(UP_POSITION);
                tongue2.setPosition(1 - UP_POSITION);
                break;

            case DOWN:
                tongue1.setPosition(DOWN_POSITION);
                tongue2.setPosition(1 - DOWN_POSITION);
                break;
        }

        // Check if movement time has elapsed
        if (moving && moveTimer.milliseconds() >= MOVE_TIME_MS) {
            moving = false;
        }
    }

    @Override
    public void stop() {
        tongue1.setPosition(DOWN_POSITION);
        tongue2.setPosition(1 - DOWN_POSITION);
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Tongue Target", targetMode);
        telemetry.addData("Tongue Moving", moving);
        telemetry.addData("Timer", moveTimer.milliseconds());
    }
}