package org.firstinspires.ftc.teamcode.drive.robot1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DiffyTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.IntakeTurretTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;

@Config
public class Robot1 extends RobotBase {


    ///////////////////* CONSTANTS *///////////////////

    public static double VERTICAL_TICKS_PER_INCH  = 943.0/29, VERTICAL_EXTEND_MAX_INCH = 29;
    public static double VERTICAL_SLIDE_TRANSFER_POSITION = 0 / VERTICAL_TICKS_PER_INCH;

    // PID + Profile Constraints
    public static PIDCoefficients VERTICAL_SLIDES_PID_COEFFICIENTS = new PIDCoefficients(0.3, 1.0, 0.015, 0.0, 0.0,0.22); // new PIDCoefficients(0.3, 1.0, 0.006, 0.0, 0.0,0.16); //0.07 min ff, 0.37 = max ff, so do
    public static ProfileConstraints DELIVERY_ARM_CONSTRAINTS = new ProfileConstraints(10, 12, 16);

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.02,
            INTAKE_ARM_TRANSFER = 0.00,
            INTAKE_ARM_HOVER = 0.15,
            INTAKE_ARM_BACK = 0.3,
            INTAKE_ARM_SLIGHTLY_UP = 0.10;

    public static double INTAKE_WRIST_TRANSFER = 0.58,
            INTAKE_WRIST_HOVER = 0.0,
            INTAKE_WRIST_DOWN = 0.04,
            INTAKE_WRIST_BACK = 0.62,
            INTAKE_WRIST_SLIGHTLY_UP = 0.48;
    public static double TURRET_DROP_OFF = 1.0,
            TURRET_STRAIGHT = 0.85,
            TURRET_LEFT_45_DEG = 0.70,
            TURRET_RIGHT_45_DEG = 0.99;
    public static double INTAKE_CLAW_CLOSE = 0.29,//0.22,
            INTAKE_CLAW_OPEN = 0.35;//0.5;
    public static double INTAKE_PIVOT_TILTED_LEFT = 0.86,//0.40,
            INTAKE_PIVOT_TILTED_RIGHT = 0.55,//0.11,
            INTAKE_PIVOT_HORIZONTAL = 0.37,//0.60,
            INTAKE_PIVOT_VERTICAL = 0.70,//0.27,
            INTAKE_PIVOT_VERTICAL_INVERSE = 0.05;
    public static double DIFFY_LEFT_TRANSFER = 0.60, DIFFY_RIGHT_TRANSFER = 0.52,
            DIFFY_LEFT_FLAT = 0.61, DIFFY_RIGHT_FLAT = 0.74,
            DIFFY_LEFT_UP = 0.56, DIFFY_RIGHT_UP = 0.57,
            DIFFY_LEFT_CLIP = 0.605, DIFFY_RIGHT_CLIP = 0.515,
            DIFFY_LEFT_DELIVER = 0.52, DIFFY_RIGHT_DELIVER = 0.60;

    public static double DELIVERY_CLAW_OPEN = 0.6, DELIVERY_CLAW_CLOSE = 0.3;

    public static double LEFT_HANG_INIT = 0.0, LEFT_HANG_ENGAGED = 0.0,
            RIGHT_HANG_INIT = 0.0, RIGHT_HANG_ENGAGED = 0.0;


    ///////////////////* HARDWARE *///////////////////


    public Robot1(HardwareMap hardwareMap) {
        super(hardwareMap);

        setDriveParams();
        super.init(hardwareMap, LocalizerMode.PINPOINT, new Pose2d(0, 0, 0));
    }

    private void setDriveParams() {

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        p.inPerTick = 1.0;
        p.lateralInPerTick = 0.851106864; // p.inPerTick;
        p.trackWidthTicks = 14.585145842107918;

        // feedforward parameters (in tick units)
        p.kS = 1.581030305278766;
        p.kV = 0.1325846626043403;
        p.kA = 0.03;

        // path profile parameters (in inches)
        p.maxWheelVel = 60;
        p.minProfileAccel = -30; //TODO change this to -40
        p.maxProfileAccel = 50; //TODO change this to 60

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 7.0;
        p.headingGain = 8.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn

        PinpointLocalizer.Params localizerParams = PinpointLocalizer.PARAMS;
        localizerParams.perpXTicks = 6.375;
        localizerParams.parYTicks = -0.15;
    }

    @Override
    public double getIntakeArmPosition(IntakeArmTask.Position position) {
        switch(position) {
            case TRANSFER:
                return INTAKE_ARM_TRANSFER;
            case HOVER:
                return INTAKE_ARM_HOVER;
            case BACK:
                return INTAKE_ARM_BACK;
            case SLIGHTLY_UP:
                return INTAKE_ARM_SLIGHTLY_UP;
            case DOWN:
            default:
                return INTAKE_ARM_DOWN;
        }
    }

    @Override
    public double getIntakePivotPosition(IntakePivotTask.Position position) {
        switch(position) {
            case TILTED_LEFT:
                return INTAKE_PIVOT_TILTED_LEFT;
            case TILTED_RIGHT:
                return INTAKE_PIVOT_TILTED_RIGHT;
            case HORIZONTAL:
                return INTAKE_PIVOT_HORIZONTAL;
            case VERTICAL_INVERSE:
                return INTAKE_PIVOT_VERTICAL_INVERSE;
            case VERTICAL:
            default:
                return INTAKE_PIVOT_VERTICAL;
        }
    }

    @Override
    public double getTurretPosition(IntakeTurretTask.Position position) {
        switch(position) {
            case DROP_OFF:
                return TURRET_DROP_OFF;
            case LEFT_45_DEG:
                return TURRET_LEFT_45_DEG;
            case RIGHT_45_DEG:
                return TURRET_RIGHT_45_DEG;
            case STRAIGHT:
            default:
                return TURRET_STRAIGHT;
        }
    }

    @Override
    public double getIntakeClawPosition(IntakeClawTask.Position position) {
        switch(position) {
            case CLOSE:
                return INTAKE_CLAW_CLOSE;
            case OPEN:
            default:
                return INTAKE_CLAW_OPEN;
        }
    }

    @Override
    public double getDeliveryClawPosition(DeliveryClawTask.Position position) {
        switch(position) {
            case CLOSE:
                return DELIVERY_CLAW_CLOSE;
            case OPEN:
            default:
                return DELIVERY_CLAW_OPEN;
        }
    }

    @Override
    public double getIntakeWristPosition(IntakeWristTask.Position position) {
        switch(position) {
            case TRANSFER:
                return INTAKE_WRIST_TRANSFER;
            case HOVER:
                return INTAKE_WRIST_HOVER;
            case DOWN:
                return INTAKE_WRIST_DOWN;
            case SLIGHTLY_UP:
                return INTAKE_WRIST_SLIGHTLY_UP;
            case BACK:
            default:
                return INTAKE_WRIST_BACK;
        }
    }

    @Override
    public double getDiffyTargetPos(DiffyTask.WhichDiffy placer, DiffyTask.Position position) {
        if(placer == DiffyTask.WhichDiffy.LEFT) {
            switch(position) {
                case TRANSFER:
                    return DIFFY_LEFT_TRANSFER;
                case FLAT:
                    return DIFFY_LEFT_FLAT;
                case UP:
                    return DIFFY_LEFT_UP;
                case CLIP:
                    return DIFFY_LEFT_CLIP;
                case DELIVER:
                default:
                    return DIFFY_LEFT_DELIVER;
            }
        }else {
            switch(position) {
                case TRANSFER:
                    return DIFFY_RIGHT_TRANSFER;
                case FLAT:
                    return DIFFY_RIGHT_FLAT;
                case UP:
                    return DIFFY_RIGHT_UP;
                case CLIP:
                    return DIFFY_RIGHT_CLIP;
                case DELIVER:
                default:
                    return DIFFY_RIGHT_DELIVER;
            }
        }
    }
    @Override
    public double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position) {
        switch (position) {
            case ENGAGED:
                return placer == HangTask.WhichHang.LEFT ? LEFT_HANG_ENGAGED : RIGHT_HANG_ENGAGED;
            case INIT:
            default:
                return placer == HangTask.WhichHang.LEFT ? LEFT_HANG_INIT : RIGHT_HANG_INIT;
        }
    }


    // Update Constants
    @Override
    public double getVerticalSlidesTicksPerInch() {
        return VERTICAL_TICKS_PER_INCH;
    }
    @Override
    public double getVerticalSlidesMaxIn() {
        return VERTICAL_EXTEND_MAX_INCH;
    }

    @Override
    public PIDCoefficients getVerticalSlidesPidCoefficients() {
        return VERTICAL_SLIDES_PID_COEFFICIENTS;
    }

    public double getVerticalSlideTransferPosition() {
        return VERTICAL_SLIDE_TRANSFER_POSITION;
    }


}
