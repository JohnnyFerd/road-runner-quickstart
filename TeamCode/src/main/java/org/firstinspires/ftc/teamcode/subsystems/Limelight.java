package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends Subsystem {

    private Limelight3A limelight;
    private Telemetry telemetry;
    private HardwareMap hwMap;

    private LLResult latestResult;

    // === Configurable ===
    public static int PIPELINE = 0;

    // === Output Values ===
    public boolean targetVisible = false;
    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public Pose3D botpose = null;

    public String patternLabel = "None";
    public String goalLabel = "None";

    public Limelight(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 30);

        telemetry.addLine("Limelight initialized");
    }

    @Override
    public void update() {
        if (limelight == null) return;

        latestResult = limelight.getLatestResult();

        targetVisible = false;
        tx = 0;
        ty = 0;
        ta = 0;
        botpose = null;
        patternLabel = "None";
        goalLabel = "None";

        if (latestResult != null && latestResult.isValid()) {
            targetVisible = true;
            tx = latestResult.getTx();
            ty = latestResult.getTy();
            ta = latestResult.getTa();
            botpose = latestResult.getBotpose();

            int id = latestResult;

            switch (id) {
                case 20:
                    goalLabel = "bluegoal";
                    break;
                case 21:
                    patternLabel = "GPP";
                    break;
                case 22:
                    patternLabel = "PGP";
                    break;
                case 23:
                    patternLabel = "PPG";
                    break;
                case 24:
                    goalLabel = "redgoal";
                    break;
            }
        }
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Target Visible", targetVisible);

        if (targetVisible) {
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("TA", ta);
            telemetry.addData("Goal Label", goalLabel);
            telemetry.addData("Pattern Label", patternLabel);

            if (botpose != null) {
                telemetry.addData("BotPose X", botpose);

            }
        } else {
            telemetry.addLine("No target detected");
        }
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public void setPipeline(int pipeline) {
        PIPELINE = pipeline;
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public Limelight3A getCamera() {
        return limelight;
    }
}