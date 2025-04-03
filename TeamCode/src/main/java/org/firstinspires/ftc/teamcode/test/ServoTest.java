package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Servo Tester", group = "Test")
@Config

public class ServoTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private V3Robot robot;
//    private GamePad gp1;

    public double  LEFTPTO_ENGAGE = 0.17;
    public double RIGHTPTO_ENGAGE = 0.86;
    public double i_CLAW_CLOSE = 0.65, i_CLAW_OPEN = 0.9;
    public double d_CLAW_CLOSE = 0.12, d_CLAW_OPEN = 0.4;

    // iwrist positions
    public double i_WRIST_OBSERVATION = 0.76;
    public double i_WRIST_TRANSFER = 0.45;
    public double i_WRIST_INIT = 0.5;
    public double i_WRIST_DOWN = 0.99;


    public double i_ARM_INIT = 0.2;
    public double i_ARM_OBSERVATION = 0.55;
    public double i_ARM_HOVER = 0.5;
    public double i_ARM_DOWN = 0.59;

    public double i_TURRET_MIDDLE = 0.26;
    public double i_TURRET_OBSERVATION = 0.9;


    public double i_PIVOT_MIDDLE = 0.32;
    public double i_PIVOT_OBSERVATION = 0.2;


    public double d_ARM_WALL_PICKUP = 0.77;
    public double d_ARM_TRANSFER = 0.2;
    public double d_ARM_DELIVER_SPECIMEN = 0.23;  // not tuned
    public double d_ARM_DELIVER_SAMPLE = 0.5; // not tuned

    public double d_PIVOT_UP = 0.26;
    public double d_PIVOT_DOWN = 0.92;


    // observation zone positions
    // arm 0.55
    // turret 0.9
    // ipivot 0.2, iwrist 0.76,

    // transfer position
    // dArmPos 0.2, iarm 0.64,  iwrist 0.45, dpivot 0.92

    // wall pick up
    // dpivot 0.26, darm 0.77

    // init pos
    // iarm 0.2, iwrist 0.5

    // hover pos
    // iarm 0.5, iwrist 0.99

    // pickup ground
    // iarm 0.59, iwrist 0.99

    Servo rightPTO, dArm, dClaw, hangHookRelease, dPivot;
    Servo leftPTO, turret, iPivot, iArm, iWrist, iClaw;
    public static double rightPTOPos = 0.92, dArmPos = 0.72, dClawPos = 0.4, hangHookPos=0.4, dPivotPos = 0.26;
    public static double leftPTOPos= 0.1, turretPos = 0.26, iPivotPos = 0.32, iArmPos = 0.58, iWristPos = 0.99, iClawPos = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        rightPTO = hardwareMap.get(Servo.class, "rightPTO");
        dArm = hardwareMap.get(Servo.class, "dArm");
        dClaw = hardwareMap.get(Servo.class, "dClaw");
        hangHookRelease = hardwareMap.get(Servo.class, "hangHookRelease");
        dPivot = hardwareMap.get(Servo.class, "dPivot");
        leftPTO = hardwareMap.get(Servo.class, "leftPTO");
        turret = hardwareMap.get(Servo.class, "turret");
        iPivot = hardwareMap.get(Servo.class, "iPivot");
        iArm = hardwareMap.get(Servo.class, "iArm");
        iWrist = hardwareMap.get(Servo.class, "iWrist");
        iClaw = hardwareMap.get(Servo.class, "iClaw");

        waitForStart();

        while (opModeIsActive()) {
            rightPTO.setPosition(rightPTOPos);
            dArm.setPosition(dArmPos);
            dClaw.setPosition(dClawPos);
            hangHookRelease.setPosition(hangHookPos);
            dPivot.setPosition(dPivotPos);
            leftPTO.setPosition(leftPTOPos);
            turret.setPosition(turretPos);
            iPivot.setPosition(iPivotPos);
            iArm.setPosition(iArmPos);
            iWrist.setPosition(iWristPos);
            iClaw.setPosition(iClawPos);
        }
    }
}
