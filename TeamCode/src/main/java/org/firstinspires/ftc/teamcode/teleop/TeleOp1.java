package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.task.DecisionTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.IntakeTurretTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;

@TeleOp(name = "TeleOp1")
@Config
public class TeleOp1 extends LinearOpMode {

    public static int pipelineNo = 6;
    private Task task = null;
    private IntakeState intakeState = IntakeState.NEUTRAL;


    private RobotBase robot;
    private GamePad gp1, gp2;

    private boolean clawOpen, intakeClawVertical = true, testToggle = false;

    public static double MAX_SLIDE_EXTENSION_INCH = 21.0, VERTICAL_STAND_POWER = 0.125;
    public static double CHAMBER_DELIVER_HEIGHT = 21, CHAMBER_DELIVER_DIST_DOWN = 8.5;
    public static int DELIVERY_CLAW_OPEN_DELAY = 200;

    private SensorUpdateThread sensorUpdateThread;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt;
    public MultipleTelemetry multipleTelemetry;
    boolean lastHang = false;


    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.limelight.pipelineSwitch(pipelineNo);
        sensorUpdateThread = new SensorUpdateThread(robot);
        robot.teleOpInit();
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        waitForStart();
        robot.limelight.start();
        sensorUpdateThread.start();

        while (opModeIsActive()) {

            driveRobot();
            runIntake();
            runDelivery();
            runHorizontalSlides();
            runHang();
            checkPreset();
            update();

            if(gp1.onceA()) {
                if(testToggle) {
                    robot.setIntakeWristPos(IntakeWristTask.Position.TRANSFER);
                } else {
                    robot.setIntakeWristPos(IntakeWristTask.Position.DOWN);
                }testToggle = !testToggle;
            }

        }
        robot.limelight.stop();
        sensorUpdateThread.interrupt();
    }
    private void checkPreset() {

    }
    private void runDelivery() {
        if (gp2.onceX()) {
            if (task != null) task.cancel();
            task = new ParallelTask(
                    new SeriesTask(
                            new SleepTask(200),
                            new VerticalSlidesTask(robot, 0,1.0, 1000)
                    ),
                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT)
            );
        }
        if (gp2.onceY()) {
            if (task != null) task.cancel();
            if (robot.getVerticalSlidesHeightInch() < 2.0) {
                task = new SeriesTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                        new ParallelTask(
                                new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT, 1.0, 1000),
                                new SeriesTask(
                                        new SleepTask(200),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP)
                                ),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                        )
                );
            } else {
                task = new ParallelTask(
                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT - CHAMBER_DELIVER_DIST_DOWN, 1.0,750),
                        new SeriesTask(
                                new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                        )
                );
            }
        }
        if(gp2.dpadUp()) {
            robot.initHang();
            robot.setVerticalSlidePowerWithoutEncoder(1.0);
        }else if(gp2.dpadDown()) {
            robot.setVerticalSlidePowerWithoutEncoder(-0.5);
        } else if(task==null) {
            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeightInch() < 2.0 ? 0 : VERTICAL_STAND_POWER);
        }
    }

    private void driveRobot() {
        double x = 0, y = 0, a = 0;
        if (gp1.dpadLeft() || gp1.dpadRight()) {
            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
        } else if (gp1.dpadUp() || gp1.dpadDown()) {
            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
        } else {
            x = -gp1.leftStickY();
            y = -gp1.leftStickX();
            a = -gp1.rightStickX() * 0.9;
        }
        double pow = Math.sqrt(x * x + y * y);
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);
        if (robot.getHorizontalSlideDistInch() > 8.0) {
            x *= 0.6;
            y *= 0.6;
            a *= 0.6;
        }

        robot.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), a));
    }

    private void runHorizontalSlides() { //TODO: check this logic
        if(gp1.onceLeftBumper()) {
            if(task!=null) {
                task.cancel();
            }
            task = new HorizontalSlidesTask(robot, 22, 1.0, 1500);
        }
    }

    private void runIntake(){
        if (gp2.onceDpadRight()) {
            if (task != null) task.cancel();
            intakeState = IntakeState.OBS_ZONE;
            task = new SeriesTask(
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.OBS_ZONE),
                            new IntakeWristTask(robot, IntakeWristTask.Position.OBS_ZONE),
                            new IntakePivotTask(robot, SampleOrientation.HORIZONTAL),
                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT),
                            new VerticalSlidesTask(robot, 0, 1.0),
                            new HorizontalSlidesTask(robot, 0, 1.0, 1500)
                    ),
                    new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE)
            );
        }
        if (gp2.onceDpadLeft()) {
            robot.setIntakeClawPos(IntakeClawTask.Position.OPEN);
        }

        if(gp1.onceRightTrigger() || gp2.onceRightTrigger()) {
            if(task != null) task.cancel();
            if(intakeState == IntakeState.NEUTRAL || intakeState == IntakeState.OBS_ZONE) {
                //go to hover
                task = createHoverTask();
                intakeClawVertical = true;
                intakeState = IntakeState.HOVER;
            } else if(intakeState == IntakeState.TRANSFER) {
                //go to neutral
                task = createNeutralTask();
                intakeClawVertical = true;
                intakeState = IntakeState.NEUTRAL;
                if(cycleTimer!=null) {
                    cycleSum += cycleTimer.seconds();
                    cycleCnt++;
                }cycleTimer = new ElapsedTime();
            } else if(intakeState == IntakeState.HOVER) {
                //try to intake
                task = createIntakeRetryTask();
                intakeState = IntakeState.HOVER;
            }
        } else if(gp1.onceLeftTrigger() || gp2.onceLeftTrigger()) {
            if(intakeState == IntakeState.HOVER) {
                if(task != null) task.cancel();
                //go to transfer
                task = createTransferTask();
                intakeClawVertical = true;
                intakeState = IntakeState.TRANSFER;
            }
        }

        if (gp1.onceA() || gp2.onceA()) {
            robot.setDeliveryClawPos(clawOpen ? DeliveryClawTask.Position.CLOSE : DeliveryClawTask.Position.OPEN);
            clawOpen = !clawOpen;
        }

        if (gp1.onceRightBumper() || gp2.onceRightBumper()) {
            robot.setIntakePivotPos(intakeClawVertical? SampleOrientation.HORIZONTAL : SampleOrientation.VERTICAL);
            intakeClawVertical = ! intakeClawVertical;
        }
    }

    private void runHang() {
        double pow = Math.min(Math.abs(gp2.leftStickY()), Math.abs(gp2.rightStickY()));
        if (pow > 0.3) {
            if (!robot.isHangEngaged()) {
                robot.engageHang();
                if (task != null) task.cancel();
                task = new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.OBS_ZONE),
                                new IntakeWristTask(robot, IntakeWristTask.Position.OBS_ZONE),
                                new IntakePivotTask(robot, SampleOrientation.HORIZONTAL),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.UP),
                                new VerticalSlidesTask(robot, 0, 1.0),
                                new HorizontalSlidesTask(robot, 0, 1.0, 1500)
                        ),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE)
                );
            }
            robot.setHangMotorPowers(pow);
            robot.setHorizontalSlidesPowerWithoutEncoder(-VERTICAL_STAND_POWER);
            lastHang = true;
        } else if (lastHang) {
            robot.setHangMotorPowers(0);
            lastHang = false;
        }
        if (gp2.onceLeftStickButton() || gp2.onceRightStickButton() ||
                gp1.onceLeftStickButton() || gp2.onceRightStickButton()) {
            robot.initHang();
        }
        if (gp2.back() && gp2.onceB()) {
            robot.releaseHang();
        }
    }

    private void update() {

//        LLStatus status = robot.limelight.getStatus();
//        telemetry.addData("Name", "%s",
//                status.getName());
//        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
//
//        Coordinates objectPosition = limelightProcessor.getObjectPosition(objectCenterHeightInch);
//        telemetry.addData("objectXPos",objectPosition.x);
//        telemetry.addData("objectYPos",objectPosition.y);
//        telemetry.addData("objectAngleRadians",objectPosition.angle);

        multipleTelemetry.addData("avg cycle time", cycleCnt==0?-1:cycleSum/cycleCnt);
        multipleTelemetry.update();


        if(task != null && task.perform())  task = null;
        gp1.update();
        gp2.update();
        robot.updateEverything();
    }

    private enum IntakeState {
        NEUTRAL,
        TRANSFER,
        HOVER,
        OBS_ZONE
    }

    private Task createHoverTask() {
        Task hoverTask = new SeriesTask(
//                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                new ParallelTask(
                        new HorizontalSlidesTask(robot, 10.0,1.0,1500),
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                        new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                        new SeriesTask(
                                new SleepTask(200),
                                new IntakeWristTask(robot, IntakeWristTask.Position.HOVER)
                        )
//                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
//                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER)
                )
        );
        return hoverTask;
    }
    private Task createIntakeRetryTask() {
        Task intakeRetryTask = new SeriesTask(
                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                new SleepTask(300),
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                        new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                ),
                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                new SleepTask(300),
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER)
                )
        );
        return intakeRetryTask;
    }
    private Task createTransferTask() {
        Task transferTask = new SeriesTask(
                new ParallelTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER),
                        new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE)
                ),
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT)
                ),
                new ParallelTask(
                        new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                        new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                        new DecisionTask(
                                new SampleStateCondition(robot),
                                new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                                new IntakePivotTask(robot, SampleOrientation.VERTICAL_INVERSE)
                        ),
                        new HorizontalSlidesTask(robot, -0.0, 1.0, 1500)
                ),
                new ParallelTask(
                        new HorizontalSlidesTask(robot, -2.0, 1.0, 700),
                        new SeriesTask(
                            new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                            new ParallelTask(
                                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                                    new SeriesTask(
                                            new SleepTask(500),
                                            new ParallelTask(
                                                    new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                                                    new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                                                    new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                                                    new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                                                    new VerticalSlidesTask(robot, 34.0,1.0,2000)
                                            )
                                    )
                            )
                        )
                )
        );
        return transferTask;
    }

    private Task createNeutralTask() {
        Task neutralTask = new SeriesTask(
                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.UP),
                        new SeriesTask(
                                new SleepTask(500),
                                new VerticalSlidesTask(robot, 0.0, 0.5,1500)
                        )
                )
        );
        return neutralTask;
    }

}
