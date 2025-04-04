package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot2.Robot2;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeTurretTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;
import org.firstinspires.ftc.teamcode.util.limelight.ObjectOrientation;

@TeleOp(name = "Robot 2 Test w/ Limelight", group = "Test")
@Config
public class Robot2Test extends LinearOpMode {

    // Base
    public Robot2 robot;
    private GamePad gamepad;
    private Task task = null;
    public MultipleTelemetry multipleTelemetry;

    // Vision

    private LimelightSampleDetector limelightSampleDetector;
    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            -30, 54,41,
            7.5,6.0,14.3,
            0.081081081, 0.12); //ROBOT 5 MEASUREMENTS
    public static int pipelineNo = 6;
    public static double distBeforeTurretCenter = 7.0, TURRET_LENGTH_INCH = 5.0;

    // Position Tuning
    public static boolean debugMode;
    public static double turretPos = Robot2.TURRET_STRAIGHT,
            pivotPos = Robot2.INTAKE_PIVOT_VERTICAL,
            intakeArmPos = Robot2.INTAKE_ARM_BACK,
            intakeWristPos = Robot2.INTAKE_WRIST_BACK,
            deliveryLeftPos = Robot2.DELIVERY_LEFT_FLAT,
            deliveryRightPos = Robot2.DELIVERY_RIGHT_FLAT,
            deliveryClawPos = Robot2.DELIVERY_CLAW_OPEN,
            intakeClawPos = Robot2.INTAKE_CLAW_OPEN,
            leftHangPos = Robot2.LEFT_HANG_INIT,
            rightHangPos = Robot2.RIGHT_HANG_INIT,
            verticalSlidePower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePad(gamepad1);
        robot = (Robot2) RobotFactory.createRobot(hardwareMap);
        limelightSampleDetector = new LimelightSampleDetector(robot.limelight, LLConfig);

        robot.limelight.pipelineSwitch(pipelineNo);
        robot.limelight.start();
        robot.teleOpInit();

//        robot.setHorizontalSlidesPower(0.3);

        waitForStart();

//        robot.setIntakeArmPos(V5IntakeArmTask.Position.BACK);
//        robot.setIntakeWristPos(V5IntakeWristTask.Position.TRANSFER);
//        robot.setIntakeClawPos(V5IntakeClawTask.Position.OPEN);
//        robot.setIntakePivotPos(V5IntakePivotTask.Position.VERTICAL);
//        robot.setDeliveryClawPos(V5DeliveryClawTask.Position.OPEN);
//        robot.setDiffyPos(V5DiffyTask.Position.FLAT);

        while (opModeIsActive()) {

            LLStatus status = robot.limelight.getStatus();
            multipleTelemetry.addData("Name", "%s",
                    status.getName());
            multipleTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            multipleTelemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            Coordinates objectPosition = limelightSampleDetector.getObjectPosition();
            multipleTelemetry.addData("objectXPos",objectPosition.x);
            multipleTelemetry.addData("objectYPos",objectPosition.y);

            ObjectOrientation orientation = limelightSampleDetector.getObjectOrientation();
            multipleTelemetry.addData("orientation",orientation.toString());

            multipleTelemetry.addData("vertical slide pos ticks", robot.getVerticalSlidesHeightTicks());
            multipleTelemetry.addData("vertical slide pos inch", robot.getVerticalSlidesHeightInch());
            multipleTelemetry.addData("horizontal slide pos ticks", robot.getHorizontalSlideDistTicks());




            if(gamepad.onceX()) {

                Coordinates invKin = inverseKinematics(objectPosition);

                double absoluteDist = Math.sqrt(objectPosition.x*objectPosition.x + objectPosition.y*objectPosition.y);

                task = new SeriesTask(
                        new ParallelTask(
                                new HorizontalSlidesTask(robot, invKin.getY(), 1.0, 1500),
                                new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                                new IntakePivotTask(robot, robot.getIntakePivotPosition(orientationToPivotPosition(orientation))+angleToPivotPosition(invKin.getAngle())),
                                new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                                new IntakeTurretTask(robot, angleToServoPosition(invKin.getAngle())),
                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                        ),
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                                new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                        ),
                        new ParallelTask(
                                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE)
                        ),
                        new SleepTask(500),
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                                new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT)
                        )
//                        new ParallelTask(
//                                new V5HorizontalSlidesTask(robot, -0.5, 1.0, 1500),
//                                new V5IntakeArmTask(robot, V5IntakeArmTask.Position.BACK),
//                                new V5IntakePivotTask(robot, V5IntakePivotTask.Position.VERTICAL),
//                                new V5IntakeWristTask(robot, V5IntakeWristTask.Position.TRANSFER),
//                                new V5TurretTask(robot, V5TurretTask.Position.STRAIGHT)
//                        )
                );
            } else if(gamepad.onceY()) {
                task = new SeriesTask(
                        new ParallelTask(
                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER)
                        ),
                        new ParallelTask(
                                new HorizontalSlidesTask(robot, 0, 1.0, 1500),
                                new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                                new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL),
                                new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT)
                        ),
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                        new ParallelTask(
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT)
                        )
                );
            }
            if(debugMode) {
                robot.deliveryLeft.setPosition(deliveryLeftPos);
                robot.deliveryRight.setPosition(deliveryRightPos);
                robot.turret.setPosition(turretPos);
                robot.intakePivot.setPosition(pivotPos);
                robot.intakeClaw.setPosition(intakeClawPos);
                robot.deliveryClaw.setPosition(deliveryClawPos);
                robot.intakeArm.setPosition(intakeArmPos);
                robot.intakeWrist.setPosition(intakeWristPos);
                robot.setVerticalSlidePowerWithoutEncoder(verticalSlidePower);
                robot.leftHang.setPosition(leftHangPos);
                robot.rightHang.setPosition(rightHangPos);
            }

            if(task != null && task.perform())  task = null;
            gamepad.update();
            robot.updateEverything();
            limelightSampleDetector.updateLimelight();

            multipleTelemetry.addData("x", robot.pose.position.x);
            multipleTelemetry.addData("y", robot.pose.position.y);
            multipleTelemetry.addData("heading (rad)", robot.pose.heading.toDouble());

            multipleTelemetry.update();
        }
        robot.limelight.stop();
    }

    private Coordinates inverseKinematics(Coordinates position) {
        position.y -= distBeforeTurretCenter;

        double y = position.y - Math.sqrt(TURRET_LENGTH_INCH*TURRET_LENGTH_INCH - position.x*position.x);
        double angle = Math.toDegrees(Math.asin(position.x/TURRET_LENGTH_INCH));
        if(position.x>TURRET_LENGTH_INCH) {
            angle = position.x>0? 45:-45;
        }

        Log.e("angle x", position.x+"");
        Log.e("angle prev", angle+"");

        Coordinates invKin = new Coordinates(0, y, angle);

        return invKin;
    }
    private double angleToServoPosition(double angle) {
        Log.e("angle", ""+angle);
        if(angle<0) {
            return Utils.clamp(Robot2.TURRET_STRAIGHT+(Robot2.TURRET_STRAIGHT - Robot2.TURRET_LEFT_45_DEG)*angle/45.0, 0.0, 1.0);
        } else {
            return Utils.clamp(Robot2.TURRET_STRAIGHT+(Robot2.TURRET_RIGHT_45_DEG - Robot2.TURRET_STRAIGHT)*angle/45.0, 0.0, 1.0);
        }
    }
    private double angleToPivotPosition(double angle) {
        Log.e("angle for pivot", ""+angle);
        if(angle>0) {
            return Utils.clamp((Robot2.INTAKE_PIVOT_TILTED_LEFT - Robot2.INTAKE_PIVOT_VERTICAL)*angle/45.0, 0.0, 1.0);
        } else {
            return Utils.clamp((Robot2.INTAKE_PIVOT_VERTICAL - Robot2.INTAKE_PIVOT_TILTED_RIGHT)*angle/45.0, 0.0, 1.0);
        }
    }
    private IntakePivotTask.Position orientationToPivotPosition(ObjectOrientation orientation) {
        switch(orientation) {
            case TILTED_LEFT:
                return IntakePivotTask.Position.TILTED_LEFT;
            case TILTED_RIGHT:
                return IntakePivotTask.Position.TILTED_RIGHT;
            case HORIZONTAL:
                return IntakePivotTask.Position.HORIZONTAL;
            case VERTICAL:
            default:
                return IntakePivotTask.Position.VERTICAL;
        }
    }


}
