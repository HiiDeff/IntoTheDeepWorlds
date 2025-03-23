package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DiffyTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeTurretTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightProcessorBase;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;
import org.firstinspires.ftc.teamcode.util.limelight.ObjectOrientation;

import java.util.List;

@TeleOp(name = "Robot 1 Test w/ Limelight", group = "Test")
@Config
public class Robot1Test extends LinearOpMode {

    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            -30, 54,41,
            7.5,6.0,15.0); //ROBOT 5 MEASUREMENTS
    private LimelightSampleDetector limelightSampleDetector;

    public Robot1 robot;
    private GamePad gamepad;
    public static int pipelineNo = 6;
    public static double distBeforeTurretCenter = 7.0;
    public static double turretPos = 0.54, pivotPos = 0.71;
    public static double TURRET_LENGTH_INCH = 5.0;
    public static double DIFFY_LEFT, DIFFY_RIGHT, deliveryClawPos, intakeClawPos;
    public static boolean debugMode;
    private Task task = null;
    public MultipleTelemetry multipleTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePad(gamepad1);
        robot = (Robot1) RobotFactory.createRobot(hardwareMap);
        limelightSampleDetector = new LimelightSampleDetector(robot.limelight, LLConfig);

        robot.limelight.pipelineSwitch(pipelineNo);
        robot.limelight.start();

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

            Coordinates objectPosition = limelightSampleDetector.getObjectPosition(1.0);
            multipleTelemetry.addData("objectXPos",objectPosition.x);
            multipleTelemetry.addData("objectYPos",objectPosition.y);

            ObjectOrientation orientation = limelightSampleDetector.getObjectOrientation();
            multipleTelemetry.addData("orientation",orientation.toString());

            limelightSampleDetector.updateLimelight();
            multipleTelemetry.update();


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
                                new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER)
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
                                new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.TRANSFER),
                                new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.TRANSFER)
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
                                new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.FLAT),
                                new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.FLAT)
                        )
                );
            }
            if(debugMode) {
                robot.diffyLeft.setPosition(DIFFY_LEFT);
                robot.diffyRight.setPosition(DIFFY_RIGHT);
                robot.turret.setPosition(turretPos);
                robot.intakePivot.setPosition(pivotPos);
                robot.intakeClaw.setPosition(intakeClawPos);
            }

            if(task != null && task.perform())  task = null;
            gamepad.update();
            robot.updateEverything();
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
            return Utils.clamp(Robot1.TURRET_STRAIGHT+(Robot1.TURRET_STRAIGHT - Robot1.TURRET_LEFT_45_DEG)*angle/45.0, 0.0, 1.0);
        } else {
            return Utils.clamp(Robot1.TURRET_STRAIGHT+(Robot1.TURRET_RIGHT_45_DEG - Robot1.TURRET_STRAIGHT)*angle/45.0, 0.0, 1.0);
        }
    }
    private double angleToPivotPosition(double angle) {
        Log.e("angle for pivot", ""+angle);
        if(angle>0) {
            return Utils.clamp((Robot1.INTAKE_PIVOT_TILTED_LEFT - Robot1.INTAKE_PIVOT_VERTICAL)*angle/45.0, 0.0, 1.0);
        } else {
            return Utils.clamp((Robot1.INTAKE_PIVOT_VERTICAL - Robot1.INTAKE_PIVOT_TILTED_RIGHT)*angle/45.0, 0.0, 1.0);
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
