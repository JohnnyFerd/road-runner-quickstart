package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Tongue extends Subsystem {

    private final Servo tongue1;
    private final Servo tongue2;
    private final Telemetry telemetry;

    /* ===== Tunables ===== */
    public static double DOWN_POSITION = 0.03;
    public static double UP_POSITION = 0.34;

    /* ===== State ===== */
    private Mode mode = Mode.DOWN;

    private enum Mode {
        UP,
        DOWN
    }

    public Tongue(String tongue1Name, String tongue2Name, HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        tongue1 = hwMap.get(Servo.class, tongue1Name);
        tongue2 = hwMap.get(Servo.class, tongue2Name);

        // Initialize positions
        tongue1.setPosition(DOWN_POSITION);
        tongue2.setPosition(1 - DOWN_POSITION);
    }

    /* ===== Commands ===== */

    // Toggle between up and down


    // Force tongue up
    public void setUp() {
        mode = Mode.UP;
        tongue1.setPosition(UP_POSITION);
        tongue2.setPosition(1 - UP_POSITION);
    }

    // Force tongue down
    public void setDown() {
        mode = Mode.DOWN;


    }

    /* ===== Update Loop ===== */

    @Override
    public void update() {
        switch (mode) {
            case UP:
                tongue1.setPosition(UP_POSITION);
                tongue2.setPosition(1 - UP_POSITION);
                break;

            case DOWN:
            default:
                tongue1.setPosition(DOWN_POSITION);
                tongue2.setPosition(1 - DOWN_POSITION);
                break;
        }
    }

    @Override
    public void stop() {
        tongue1.setPosition(DOWN_POSITION);
        tongue2.setPosition(1 - DOWN_POSITION);
    }

    /* ===== Telemetry ===== */

    @Override
    public void addTelemetry() {
        telemetry.addLine("Tongue Servo");
        telemetry.addData("Mode", mode);
        telemetry.addData("Tongue1 Pos", tongue1.getPosition());
        telemetry.addData("Tongue2 Pos", tongue2.getPosition());
    }
}