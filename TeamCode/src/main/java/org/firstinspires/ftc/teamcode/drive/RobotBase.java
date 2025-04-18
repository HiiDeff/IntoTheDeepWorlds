package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.IntakeTurretTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;

@Config
public abstract class RobotBase extends PinpointDrive {


    ///////////////////* CONSTANTS *///////////////////

    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2, VERTICAL_EXTEND_MAX_IN = 30.0; //actually 29
    public static double VERTICAL_TICKS_PER_INCH = 938.0/29;
    public static double VERTICAL_SLIDE_DOWN_POWER_TELEOP = -1.0, VERTICAL_SLIDE_DOWN_POWER_AUTO = -1.0;

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MIN_IN = -2.0, HORIZONTAL_EXTEND_MAX_IN = 21.5;
    public static double HORIZONTAL_TICKS_PER_INCH = 690/21.5;

    public static double RELEASE_HANG_POS = 0, STOW_HANG_POS = 1.0;

    // Intake Constants


    ///////////////////* HARDWARE *///////////////////


    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    // Servos
    public final Servo deliveryLeft, deliveryRight, deliveryClaw, turret, intakeArm, intakePivot, intakeWrist, intakeClaw;
    public final Servo leftHang, rightHang, hangHookRelease;

    // Motion PIDs + Profilers
//    public static PIDCoefficients VERTICAL_SLIDES_PID_COEFFICIENTS = new PIDCoefficients(0.3, 1.0, 0.02, 0.0, 0.0,0.22); // new PIDCoefficients(0.3, 1.0, 0.006, 0.0, 0.0,0.16); //0.07 min ff, 0.37 = max ff, so do
    public final VerticalSlidesPID verticalSlidesPID;

    // Lights
//    private final RevBlinkinLedDriver light;

    // Vision
    public final Limelight3A limelight;
    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            -30, 54,41,
            3.75,0.9,10.1,
            0.4/4.6, 1.5/10.5);
    private final LimelightSampleDetector limelightSampleDetector;
    public static int LIMELIGHT_YELLOW_PIPELINE_IDX = 5,
            LIMELIGHT_RED_PIPELINE_IDX = 6,
            LIMELIGHT_BLUE_PIPELINE_IDX = 7;
    public static double TURRET_DIST_FROM_ROBOT_CENTER = 6.0;


    //Sensors

    public final RevColorSensorV3 intakeColorSensor;

    public final IntakeSampleState intakeSampleState;

    // Cached Encoder Values
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;
    private int lastHorizontalSlidesPos = 0;

    // States
    private boolean verticalSlidesRunToPosition = false;

    public boolean hangEngaged = false;

    public RobotBase(HardwareMap hardwareMap) {

        super(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        horizontalSlides.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(verticalSlidesLeftIsReversed()) {
            verticalSlidesLeft.setDirection(DcMotor.Direction.REVERSE);
        } else {
            verticalSlidesRight.setDirection(DcMotor.Direction.REVERSE);
        }

        verticalSlidesPID = new VerticalSlidesPID(this, getVerticalSlidesPidCoefficients());

        // Servos:
        deliveryLeft = hardwareMap.get(Servo.class, "deliveryL");
        deliveryRight = hardwareMap.get(Servo.class, "deliveryR");
        deliveryClaw = hardwareMap.get(Servo.class, "dClaw");
        turret = hardwareMap.get(Servo.class, "iTurret");
        intakeArm = hardwareMap.get(Servo.class, "iArm");
        intakePivot = hardwareMap.get(Servo.class, "iPivot");
        intakeWrist = hardwareMap.get(Servo.class, "iWrist");
        intakeClaw = hardwareMap.get(Servo.class, "iClaw");
        leftHang = hardwareMap.get(Servo.class, "hangL");
        rightHang = hardwareMap.get(Servo.class, "hangR");
        hangHookRelease = hardwareMap.get(Servo.class, "hangHookRelease");

        // Lights:
//        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        // Limelight:
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSampleDetector = new LimelightSampleDetector(limelight, LLConfig);

        // Color Sensors:
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "iColor");
        intakeSampleState = new IntakeSampleState(this);

        // Update Constants

        VERTICAL_TICKS_PER_INCH = getVerticalSlidesTicksPerInch();
        VERTICAL_EXTEND_MAX_IN = getVerticalSlidesMaxIn();
    }



    ///////////////////* INIT *///////////////////

    public void teleOpInit() {
        setIntakeArmPos(IntakeArmTask.Position.BACK);
        setIntakeWristPos(IntakeWristTask.Position.TRANSFER);
        setIntakeClawPos(IntakeClawTask.Position.OPEN);
        setIntakePivotPos(SampleOrientation.VERTICAL);
        setDeliveryClawPos(DeliveryClawTask.Position.OPEN);
        setDeliveryPos(DeliveryTask.Position.FLAT);
        setTurretPos(IntakeTurretTask.Position.STRAIGHT);
        initHang();
        stowHang();
    }
    public void autoInit() {
        setIntakeArmPos(IntakeArmTask.Position.BACK);
        setIntakeWristPos(IntakeWristTask.Position.TRANSFER);
        setIntakeClawPos(IntakeClawTask.Position.OPEN);
        setIntakePivotPos(SampleOrientation.VERTICAL);
        setDeliveryClawPos(DeliveryClawTask.Position.CLOSE);
        setDeliveryPos(DeliveryTask.Position.INIT);
        setTurretPos(IntakeTurretTask.Position.STRAIGHT);
        initHang();
        stowHang();
    }

    ///////////////////* UPDATES *///////////////////

    public void updateEverything() {
        updateEncoders();
        updatePoseEstimate();
        updateProfilers();
        updatePIDs();
//        updateIntakeSampleState();
//        updateLimelight();
    }
    public void updatePIDs() {
        if(verticalSlidesRunToPosition) {
            setVerticalSlidePowerForPID(getVerticalSlidesPidCoefficients().feedForward + verticalSlidesPID.getPower());
        }
    }
    public void updateEncoders() {
        verticalSlidesPos = verticalSlidesLeft.getCurrentPosition();
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }
    public void updateLastHorizontalEncoder() {
        lastHorizontalSlidesPos = horizontalSlidesPos;
    }
    public void updateProfilers() {

    }
    public void updateIntakeSampleState() {
        intakeSampleState.update();
    }


    ///////////////////* GENERAL UTILS *///////////////////

    public void resetHorizontalAndVerticalSlidesEncoderPositions() {
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    ///////////////////* VERTICAL SLIDES UTILS *///////////////////

    public abstract boolean verticalSlidesLeftIsReversed();
    public abstract double getVerticalSlidesTicksPerInch();
    public abstract double getVerticalSlidesMaxIn();
    public abstract PIDCoefficients getVerticalSlidesPidCoefficients();
    public void setVerticalSlidePowerForPID(double power) {
        verticalSlidesLeft.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_AUTO, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_AUTO, 1.0));
    }
    public void setVerticalSlidePowerWithoutEncoder(double power) {
        verticalSlidesRunToPosition = false;
        verticalSlidesLeft.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_TELEOP, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_TELEOP, 1.0));
    }
    public void setVerticalSlidesHeight(double posInches) {
        verticalSlidesRunToPosition = true;
        verticalSlidesPID.setTargetPosition((int) (posInches * VERTICAL_TICKS_PER_INCH));
    }
    public double getVerticalSlidesHeightInch() {
        return verticalSlidesPos/VERTICAL_TICKS_PER_INCH;
    }
    public int getVerticalSlidesHeightTicks() {
        return verticalSlidesLeft.getCurrentPosition();
    }


    ///////////////////* HORIZONTAL SLIDES UTILS *///////////////////

    public void setHorizontalSlidesPower(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        horizontalSlides.setPower(power);
    }
    public void setHorizontalSlidesPowerWithoutEncoder(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        horizontalSlides.setPower(power);
    }
    public void brakeHorizontalSlides(double brakePower) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition(lastHorizontalSlidesPos);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(brakePower);
    }
    public void setHorizontalSlideDist(double posInches, double power) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
                        * HORIZONTAL_TICKS_PER_INCH)));
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(power);
    }
    public int getLastHorizontalSlideDist() {
        return lastHorizontalSlidesPos;
    }
    public double getHorizontalSlideDistInch() {
        return ((double) horizontalSlides.getCurrentPosition()) / HORIZONTAL_TICKS_PER_INCH;
    }
    public double getHorizontalSlideDistTicks() {
        return horizontalSlides.getCurrentPosition();
    }


    ///////////////////* DELIVERY UTILS *///////////////////




    ///////////////////* CLAW UTILS *///////////////////


    ///////////////////* INTAKE UTILS *///////////////////

    public abstract double getIntakeArmPosition(IntakeArmTask.Position position);
    public void setIntakeArmPos(IntakeArmTask.Position position) {
        intakeArm.setPosition(getIntakeArmPosition(position));
    }
    public abstract double getIntakePivotPosition(SampleOrientation position);
    public void setIntakePivotPos(SampleOrientation position) {
        intakePivot.setPosition(getIntakePivotPosition(position));
    }
    public abstract double getTurretPosition(IntakeTurretTask.Position position);
    public void setTurretPos(IntakeTurretTask.Position position) {
        turret.setPosition(getTurretPosition(position));
    }
    public abstract double getIntakeClawPosition(IntakeClawTask.Position position);
    public void setIntakeClawPos(IntakeClawTask.Position position) {
        intakeClaw.setPosition(getIntakeClawPosition(position));
    }
    public abstract double getIntakeWristPosition(IntakeWristTask.Position position);
    public void setIntakeWristPos(IntakeWristTask.Position position) {
        intakeWrist.setPosition(getIntakeWristPosition(position));
    }

    public abstract double getDeliveryClawPosition(DeliveryClawTask.Position position);
    public void setDeliveryClawPos(DeliveryClawTask.Position position) {
        deliveryClaw.setPosition(getDeliveryClawPosition(position));
    }

    ///////////////////* DELIVERY UTILS *///////////////////

    public abstract double getDeliveryTargetPos(DeliveryTask.WhichDelivery placer, DeliveryTask.Position position);
    public void setDeliveryPos(DeliveryTask.Position position) {
        DeliveryTask.setDeliveryPosition(this, DeliveryTask.WhichDelivery.LEFT, position);
        DeliveryTask.setDeliveryPosition(this, DeliveryTask.WhichDelivery.RIGHT, position);
    }

    ///////////////////* COLOR SENSOR + LIGHTS UTILS *///////////////////

    public boolean isSampleDetected() {
        return intakeSampleState.isSampleIn();
    }
//    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
//        light.setPattern(pattern);
//    }

    ///////////////////* HANG UTILS *///////////////////

    public abstract double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position);
    public void initHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.INIT);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.INIT);
        hangEngaged = false;
    }
    public void engageHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.ENGAGED);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.ENGAGED);
        hangEngaged = true;
    }
    public boolean isHangEngaged(){
        return hangEngaged;
    }

    public void setHangMotorPowers(double hangDrivetrainPow, double hangSlidesPow) {
        // pow should be non-negative
        rightFront.setPower(-hangDrivetrainPow);
        leftFront.setPower(-hangDrivetrainPow);
        setVerticalSlidePowerWithoutEncoder(-hangSlidesPow);
    }

    public void releaseHang() {
        hangHookRelease.setPosition(RELEASE_HANG_POS);
    }

    public void stowHang() {
        hangHookRelease.setPosition(STOW_HANG_POS);
    }

    ///////////////////* LIMELIGHT UTILS *///////////////////
    public void setLimelightPipeline(IntakeSampleState.SampleColor sampleColor) {
        switch(sampleColor) {
            case COLOR_RED:
                limelight.pipelineSwitch(LIMELIGHT_RED_PIPELINE_IDX);
                break;
            case COLOR_BLUE:
                limelight.pipelineSwitch(LIMELIGHT_BLUE_PIPELINE_IDX);
                break;
            case COLOR_YELLOW:
            default:
                limelight.pipelineSwitch(LIMELIGHT_YELLOW_PIPELINE_IDX);
                break;
        }
    }
    public void startLimelight() {
        limelight.start();
    }
    public void updateLimelight() {
        limelightSampleDetector.updateLimelight();
    }
    public void sortLimelightEntries() { limelightSampleDetector.sortCoordinates(); }
    public void stopLimelight() {
        limelight.stop();
    }
    public double getAngleToSample(Coordinates pos) {
        return Math.atan(pos.getX()/(TURRET_DIST_FROM_ROBOT_CENTER+pos.getY()));
    }
    public Coordinates getLimelightObjectPosition() {
        return limelightSampleDetector.getObjectPosition();
    }
    public SampleOrientation getLimelightObjectOrientation() {
        return limelightSampleDetector.getObjectOrientation();
    }
    public void setLimelightMaxDetectionDistInch(double dist){
        limelightSampleDetector.setMaxDetectableDistInch(dist);
    }
}
