package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;

@Config
@TeleOp(name="Comp Opmode Smart", group="TeleOp")
public class March1stOpmodeSmart extends LinearOpMode {

    /* ---------------- SUBSYSTEMS ---------------- */
    Drivetrain drivetrain;
    MecanumDrive rrDrive;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    JVBoysSoccerRobot robot;
    Tongue tongue;

    /* ---------------- VISION ---------------- */
    Limelight3A limelight;
    String goalLabel = "";
    String patternLabel = "";

    /* ---------------- GAMEPAD TRACKING ---------------- */
    Gamepad curr1 = new Gamepad();
    Gamepad prev1 = new Gamepad();
    Gamepad curr2 = new Gamepad();
    Gamepad prev2 = new Gamepad();

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean tongueUp = false;
    boolean shooterOn = false;
    boolean highRPM = false;

    public static int LOW_RPM = 2500;
    public static int HIGH_RPM = 3800;

    /* ============================================================ */
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        drivetrain = new Drivetrain(hardwareMap, telemetry, robot);
        intake = robot.intake;
        turret = robot.turret;
        spindexer = robot.spindexer;
        tongue = robot.Tongue;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        rrDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            updateGamepads();

            handleDrive();
            handleIntake();
            handleSpindexer();
            updatePose();
            updateVision();
            turret.setAim(curr1.x);  // only aim if 'a' is pressed
            robot.update(true,true);

            sendTelemetry();
        }
    }

    /* ============================================================
                           CONTROLS
       ============================================================ */
    private void handleDrive() {
        double speedScale = 1.0;
        if(curr1.left_trigger > .2) speedScale = .3;

        drivetrain.moveXYR(
                curr1.left_stick_x * 1.05 * speedScale,
                -curr1.left_stick_y * speedScale,
                curr1.right_stick_x * speedScale
        );
    }

    private void handleIntake() {

        if(curr1.right_bumper && !prev1.right_bumper){
            intakeOn = !intakeOn;
        }

        if(curr1.left_bumper){
            intake.intakeReverse();
        } else if(intakeOn){
            intake.intakeOn();
        } else {
            intake.intakeOff();
        }

        // Tongue toggle, only if spindexer is idle
        if(curr1.b && !prev1.b && spindexer.isIdle()){
            tongueUp = !tongueUp;
            if(tongueUp) tongue.setUp();
            else tongue.setDown();
        }

        // Shooter toggle
        if(curr1.dpad_up && !prev1.dpad_up){
            shooterOn = !shooterOn;
            if(shooterOn){
                int target = highRPM ? HIGH_RPM : LOW_RPM;
                robot.outake.setPresetVelocity(target);
                robot.outake.intakeOn();
            } else {
                robot.outake.intakeOff();
            }
        }

        // Toggle RPM mode
        if(curr1.y && !prev1.y){
            highRPM = !highRPM;
            if(shooterOn){
                int target = highRPM ? HIGH_RPM : LOW_RPM;
                robot.outake.setPresetVelocity(target);
            }
        }

        // Emergency stop
        if(curr1.dpad_down && !prev1.dpad_down){
            shooterOn = false;
            robot.outake.intakeOff();
        }

        // Rehome spindexer, only if tongue is down and not moving
        if(curr1.right_stick_button && !prev1.right_stick_button && tongue.isDown()){
            spindexer.rehome();
        }
    }

    private void handleSpindexer() {
        // Only move if tongue is down and spindexer is idle
        if(curr1.dpad_right && !prev1.dpad_right && tongue.isDown()){
            spindexer.rotateByFraction(1.0/3.0);
        }
        if(curr1.dpad_left && !prev1.dpad_left && tongue.isDown()){
            spindexer.rotateByFraction(-1.0/3.0);
        }
    }

    /* ============================================================
                           POSE + RPM
       ============================================================ */
    double distance = 0;
    double CalcRPMs = 0;

    private void updatePose(){
        rrDrive.updatePoseEstimate();
        Pose2d pose = rrDrive.localizer.getPose();

        distance = Math.abs(pose.position.x);
        distance = Math.max(0, Math.min(distance, 200));

        CalcRPMs = 2500 + 20 * distance;
    }

    /* ============================================================
                           LIMELIGHT
       ============================================================ */
    private void updateVision(){
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if(tags != null && !tags.isEmpty()){
                for(LLResultTypes.FiducialResult tag : tags){
                    switch(tag.getFiducialId()){
                        case 20: goalLabel="Blue"; break;
                        case 24: goalLabel="Red"; break;
                        case 21: patternLabel="GPP"; break;
                        case 22: patternLabel="PGP"; break;
                        case 23: patternLabel="PPG"; break;
                    }
                }
            }
        }
    }

    /* ============================================================
                           UTIL
       ============================================================ */
    private void updateGamepads(){
        prev1.copy(curr1);
        prev2.copy(curr2);
        curr1.copy(gamepad1);
        curr2.copy(gamepad2);
    }

    private void sendTelemetry(){
        telemetry.addData("Goal",goalLabel);
        telemetry.addData("Pattern",patternLabel);
        telemetry.addData("Distance",distance);
        telemetry.addData("RPM",CalcRPMs);
        telemetry.addData("Heading",robot.getHeadingDeg());
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("RPM Mode", highRPM ? "HIGH (3800)" : "LOW (2500)");
        telemetry.addData("Tongue Up", tongueUp);
        telemetry.addData("Tongue Moving", tongue.isBusy());
        telemetry.addData("Spindexer Moving", !spindexer.isIdle());
        telemetry.update();
    }
}