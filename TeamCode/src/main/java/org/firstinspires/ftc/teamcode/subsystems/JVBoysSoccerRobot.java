package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;

import java.util.Arrays;
import java.util.List;

/**
 * JVBoysSoccerRobot is the robot base superclass.
 * All hardware and subsystems are initialized here.
 * GO JV BOYS SOCCER TEAM!
 */
public class JVBoysSoccerRobot {

    public HardwareMap hwMap;
    public Telemetry telemetry;
    public BulkReading BR;
    private List<LynxModule> allHubs;
    private List<Subsystem> subsystems;
    public IMU imu;

    // Subsystems
    public Drivetrain drivetrainSubsystem;



    public Spindexer spindexer;

    public Intake intake;
   public Turret turret;
   public LimelightSub limelight;


    // Hardware
    public DcMotorEx motorFL, motorFR, motorBL, motorBR, motorSPINDEX, motorTURRET, motor1, motor2;


    private int hertzCounter = 0;
    private double previousTime = 0;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // Configuring Hubs to auto mode for bulk reads
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        initIMU();
        initHardware();

        drivetrainSubsystem = new Drivetrain(hwMap, telemetry, this);
        intake = new Intake("intake", hwMap, telemetry);
        //spindexer = new Spindexer("spindexer", "colorsensor", hwMap, telemetry);
        turret = new Turret(this);
        spindexer = new Spindexer(
                "spindexerServo",
                "BL",
                "colorsensor",
                hwMap,
                telemetry
        );
        limelight = new LimelightSub(hwMap, telemetry);






        if (RobotSettings.STORE_POSE) {
            drivetrainSubsystem.initYaw = RobotSettings.POSE_STORAGE;
            RobotSettings.STORE_POSE = false;
        }else {
            drivetrainSubsystem.resetInitYaw();
        }
        telemetry.addData("INIT YAW: ", drivetrainSubsystem.initYaw);

        subsystems = Arrays.asList(drivetrainSubsystem, intake, turret, spindexer );
        BR = new BulkReading(this);

    }

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry, boolean isAuto) {
        if (isAuto) {
            this.hwMap = hwMap;
            this.telemetry = telemetry;

            RobotSettings.STORE_POSE = true;

            // Configuring Hubs to auto mode for bulk reads
            allHubs = hwMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }


            //spindexer = new Spindexer("spindexer", "colorsensor", hwMap, telemetry);
            intake = new Intake("intake", hwMap, telemetry);
            limelight = new LimelightSub(hwMap, telemetry);
            turret = new Turret(this);
            spindexer = new Spindexer(
                    "spindexerServo",
                    "BL",
                    "colorsensor",
                    hwMap,
                    telemetry
            );
            initIMU();
            initHardware();
            drivetrainSubsystem = new Drivetrain(hwMap, telemetry, this);

            subsystems = Arrays.asList(drivetrainSubsystem, intake, turret, spindexer );

            RobotSettings.POSE_STORAGE = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("PoseStorage: ", RobotSettings.POSE_STORAGE);
            BR = new BulkReading(this, true);
        }
    }

    public void initIMU() {
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters1 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotSettings.LOGO_FACING_DIR, RobotSettings.USB_FACING_DIR));
        imu.initialize(parameters1);
    }

    public void initHardware() {
        initDrivetrainHardware();
        initShooterHardware();
        initTurretHardware();

    }

    public void initDrivetrainHardware() {
        motorFL = hwMap.get(DcMotorEx.class, RobotSettings.FL_NAME);
        motorBL = hwMap.get(DcMotorEx.class, RobotSettings.BL_NAME);
        motorFR = hwMap.get(DcMotorEx.class, RobotSettings.FR_NAME);
        motorBR = hwMap.get(DcMotorEx.class, RobotSettings.BR_NAME);

        motorFL.setDirection(RobotSettings.FL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorFR.setDirection(RobotSettings.FR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorBR.setDirection(RobotSettings.BR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motorBL.setDirection(RobotSettings.BL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void initTurretHardware() {
        motorTURRET = hwMap.get(DcMotorEx.class, "turretspinner");

        motorTURRET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    public void initShooterHardware(){
    //motor1 = hwMap.get(DcMotorEx.class, "spin1");
        //motor2 = hwMap.get(DcMotorEx.class, "spin2");
    }






    public void addTelemetry() {
        if (UseTelemetry.ROBOT_TELEMETRY) {
            for (Subsystem s : subsystems) {
                s.addTelemetry();
            }
        }
    }
    public double getHeadingDeg() {
        return imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;
    }

    public void update(boolean updateSubsystems, boolean useTelemetry) {
        if (updateSubsystems) {
            for (Subsystem s : subsystems) {
                s.update();
            }
        }
        if (useTelemetry) {
            addTelemetry();
        }
        hertzCounter++;
        if (RobotSettings.SUPER_TIME.seconds() - previousTime > 1.0) {
            telemetry.addData("HERTZ: ", hertzCounter);
            hertzCounter = 0;
        }
        previousTime = RobotSettings.SUPER_TIME.seconds();
        telemetry.update();
        BR.readAll();
    }
}
