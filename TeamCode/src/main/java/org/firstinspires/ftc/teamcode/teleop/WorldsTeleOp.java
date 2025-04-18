package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.common.InvKin;
import org.firstinspires.ftc.teamcode.drive.IntakeSampleState;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.DecisionTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
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
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;

@Config
public abstract class WorldsTeleOp extends LinearOpMode {

    private Task intakeTask = null, deliveryTask = null;
    private IntakeState intakeState = IntakeState.NEUTRAL;
    private boolean clawOpen, intakeClawVertical = true, testToggle = false, horizontalSlidesClose = false;
    public static double VERTICAL_STAND_POWER = 0.15, MAX_VERTICAL_SLIDE_HEIGHT_INCH = 32.0,
            CHAMBER_DELIVER_HEIGHT = 19.5, CHAMBER_DELIVER_DIST_DOWN = 8.5, HANG_HEIGHT_VERTICAL_SLIDES = 16.5, HANG_VERTICAL_SLIDES_REST_ON_BAR = 3.0;//rest on bar position = 125 ticks or 3.8265 inches
    public static double HANG_DRIVETRAIN_POWER = 1.0, HANG_SLIDE_POWER = 1.0;
    public static int DELIVERY_CLAW_OPEN_DELAY = 750;
    public static double AUTO_SPEC_Y = 29, AUTO_SPEC_X = 30;
    private RobotBase robot;
    private GamePad gp1, gp2;

    private SensorUpdateThread sensorUpdateThread;
    private ElapsedTime cycleTimer;
    private double cycleSum, cycleCnt;
    public MultipleTelemetry multipleTelemetry;
    boolean lastHang = false;


    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.setLimelightPipeline(IntakeSampleState.SampleColor.COLOR_YELLOW);
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
            update();

        }
        robot.limelight.stop();
        sensorUpdateThread.interrupt();
    }
    private void driveRobot() {
        double x = 0, y = 0, a = 0;
//        if (gp1.dpadLeft() || gp1.dpadRight()) {
//            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
//        } else if (gp1.dpadUp() || gp1.dpadDown()) {
//            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
//        } else {
            x = -gp1.leftStickY();
            y = -gp1.leftStickX();
            a = -gp1.rightStickX() * 0.9;
//        }
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
    private void runDelivery() {
        if (gp2.onceY()) {
            if (deliveryTask != null) deliveryTask.cancel();
            if (robot.getVerticalSlidesHeightInch() < 2.0) {
                if(!gp2.back()){
                    deliveryTask = new ParallelTask(
                            new HorizontalSlidesTask(robot, 0.0, 1.0,500),
                            new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                            new SeriesTask(
                                    new SleepTask(200),
                                    new ParallelTask(
                                            new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT, 1.0, 1000),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                                    )
                            )
                    );
                }else {
                    deliveryTask = createAutoChamberScoreTask();
                }
            } else {
                deliveryTask = new SeriesTask(
                        new ParallelTask(
                            new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT - CHAMBER_DELIVER_DIST_DOWN, 1.0,750),
                            new SeriesTask(
                                    new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                            )
                        )
                );
            }
        }


        if (gp2.onceA()) {
            if (deliveryTask != null) deliveryTask.cancel();
            deliveryTask = new ParallelTask(
                    new SeriesTask(
                            new SleepTask(200),
                            new VerticalSlidesTask(robot, 0,1.0, 1000)
                    ),
                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                    new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT),
                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
            );
        }
        if(gp2.dpadUp() && robot.getVerticalSlidesHeightInch()<MAX_VERTICAL_SLIDE_HEIGHT_INCH) {
            robot.initHang();
            robot.setVerticalSlidePowerWithoutEncoder(1.0);
            if(deliveryTask==null && robot.getVerticalSlidesHeightInch()<2.0) {
                deliveryTask = new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                        new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                        new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                        new HorizontalSlidesTask(robot, 0.0, 1.0, 1000)
                );
                intakeState = IntakeState.NEUTRAL;
            }
        }else if(gp2.dpadDown()&&robot.getVerticalSlidesHeightInch()>0.5) {
            robot.setVerticalSlidePowerWithoutEncoder(-0.5);
            if(deliveryTask==null && robot.getVerticalSlidesHeightInch()>27.0) {
                deliveryTask = new ParallelTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.UP)
                );
            }
        } else if(deliveryTask==null) {
            robot.setVerticalSlidePowerWithoutEncoder(robot.getVerticalSlidesHeightInch() < 2.0 ? 0 : VERTICAL_STAND_POWER);
        }

        if (gp1.onceA()) {
            robot.setDeliveryClawPos(clawOpen ? DeliveryClawTask.Position.CLOSE : DeliveryClawTask.Position.OPEN);
            clawOpen = !clawOpen;
        }
    }

    private void runHorizontalSlides() {
        if(gp1.onceLeftBumper()) {
            if(intakeTask!=null) {
                intakeTask.cancel();
            } if(horizontalSlidesClose) {
                intakeTask = new HorizontalSlidesTask(robot, 22, 1.0, 1500);
                horizontalSlidesClose = false;
            } else {
                intakeTask = new HorizontalSlidesTask(robot, 10, 1.0, 1500);
                horizontalSlidesClose = true;
            }
        }
    }

    private void runIntake(){
        if(gp1.onceY()) {
           robot.setLimelightPipeline(IntakeSampleState.SampleColor.COLOR_YELLOW);
        } else if(gp1.onceB()) {
            robot.setLimelightPipeline(isRed()? IntakeSampleState.SampleColor.COLOR_RED:IntakeSampleState.SampleColor.COLOR_BLUE);
        }

        if (gp1.onceDpadRight()) {
            if(intakeTask != null) intakeTask.cancel();
            intakeState = IntakeState.OBS_ZONE;
            intakeTask = new ParallelTask(
                    new SeriesTask(
                            new ParallelTask(
                                    new IntakeArmTask(robot, IntakeArmTask.Position.OBS_ZONE),
                                    new IntakeWristTask(robot, IntakeWristTask.Position.OBS_ZONE),
                                    new IntakePivotTask(robot, SampleOrientation.TILTED_RIGHT)
                            ),
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE_READY)
                    ),
                    new HorizontalSlidesTask(robot, 0, 1.0, 1500)
            );
        } else if (gp2.onceDpadLeft()||gp1.onceDpadLeft()) {
            if(intakeTask != null) intakeTask.cancel();
            intakeState = IntakeState.NEUTRAL;
            intakeTask = new ParallelTask(
                    new SeriesTask(
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                            new SleepTask(300),
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                            new ParallelTask(
                                    new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                                    new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                                    new IntakePivotTask(robot, SampleOrientation.VERTICAL)
                            )
                    ),
                    new HorizontalSlidesTask(robot, 0, 1.0, 1500)
            );
        }

        if(gp1.onceX() && intakeState == IntakeState.HOVER) {
            //go to transfer
            if(intakeTask != null) intakeTask.cancel();
            intakeTask = createTransferTask();
            intakeClawVertical = true;
            intakeState = IntakeState.TRANSFER;
        }

        if(gp2.onceRightTrigger()) {
            if(intakeState == IntakeState.NEUTRAL) {
                if(intakeTask != null) intakeTask.cancel();
                //go to ready
                intakeTask = createReadyTask();
                intakeClawVertical = true;
                intakeState = IntakeState.READY;
            }
            else if(intakeState == IntakeState.READY || intakeState == IntakeState.TRANSFER) {
                if(intakeTask != null) intakeTask.cancel();
                //go to hover
                if(gp2.a()) {
                    intakeTask = createAutoIntakeTask();
                    horizontalSlidesClose = false;
                    intakeClawVertical = false;
                }else {
                    intakeTask = createHoverTask();
                    horizontalSlidesClose = true;
                    intakeClawVertical = true;
                }
                intakeState = IntakeState.HOVER;
            }
        } else if(gp2.onceLeftTrigger()) {
            if(intakeState == IntakeState.HOVER) {
                if(intakeTask != null) intakeTask.cancel();
                //go to ready
                intakeTask = createReadyTask();
                intakeClawVertical = true;
                intakeState = IntakeState.READY;
            } else {
                if(intakeTask != null) intakeTask.cancel();
                intakeTask = createNeutralTask();
                intakeClawVertical = true;
                intakeState = IntakeState.NEUTRAL;
            }
        }
        if (gp1.onceRightTrigger()) {
            if(intakeState == IntakeState.HOVER) {
                if(intakeTask != null) intakeTask.cancel();
                //try to intake
                intakeTask = createIntakeRetryTask();
                intakeState = IntakeState.HOVER;
            }
        }

        if (gp1.onceRightBumper()) {
            if(!gp1.back()){
                robot.setIntakePivotPos(intakeClawVertical? SampleOrientation.HORIZONTAL : SampleOrientation.VERTICAL);
                intakeClawVertical = ! intakeClawVertical;
            } else {
                robot.resetHorizontalAndVerticalSlidesEncoderPositions();
            }
        }

        if(intakeTask==null){
            if(gp1.dpadUp()) robot.setHorizontalSlidesPowerWithoutEncoder(1.0);
            else if(gp1.dpadDown()) robot.setHorizontalSlidesPowerWithoutEncoder(-1.0);
            else if(deliveryTask==null) robot.setHorizontalSlidesPowerWithoutEncoder(0);
        }
    }

    private void runHang() {
        double pow = Math.min(Math.abs(gp2.leftStickY()), Math.abs(gp2.rightStickY()));
        if(-gp2.leftStickY()>0.5 && -gp2.rightStickY()>0.5) {
            robot.initHang();
            robot.setVerticalSlidePowerWithoutEncoder(pow);
        }else if(-gp2.leftStickY()<-0.5 && -gp2.rightStickY()<-0.5) {
            if(!(gp2.leftTrigger()>0.3&&gp2.rightTrigger()>0.3) || robot.getVerticalSlidesHeightInch()>HANG_VERTICAL_SLIDES_REST_ON_BAR) {
                if (!robot.isHangEngaged()) {
                    robot.engageHang();
                }
                robot.setHangMotorPowers(HANG_DRIVETRAIN_POWER, HANG_SLIDE_POWER);
                robot.setHorizontalSlidesPowerWithoutEncoder(-VERTICAL_STAND_POWER);
                lastHang = true;
            }
        } else if (lastHang) {
            robot.setHangMotorPowers(0, 0);
            lastHang = false;
        }
        if (gp2.onceLeftStickButton() || gp2.onceRightStickButton() ||
                gp1.onceLeftStickButton() || gp1.onceRightStickButton()) {
            robot.initHang();
        }
        if (gp2.back() && gp2.onceB()) {
            robot.initHang();
            robot.releaseHang();
            if (deliveryTask != null) deliveryTask.cancel();
            deliveryTask = new SeriesTask(
                    new SleepTask(500),
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.OBS_ZONE),
                            new IntakeWristTask(robot, IntakeWristTask.Position.OBS_ZONE),
                            new IntakePivotTask(robot, SampleOrientation.HORIZONTAL),
                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.UP),
                            new VerticalSlidesTask(robot, HANG_HEIGHT_VERTICAL_SLIDES, 1.0),
                            new HorizontalSlidesTask(robot, 0, 1.0, 1500)
                    ),
                    new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE)
            );
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
        multipleTelemetry.addData("vertical slides pos", robot.getVerticalSlidesHeightInch());
        multipleTelemetry.addData("avg cycle time", cycleCnt==0?-1:cycleSum/cycleCnt);
        multipleTelemetry.update();


        if(intakeTask != null && intakeTask.perform()) intakeTask = null;
        if(deliveryTask != null && deliveryTask.perform()) deliveryTask = null;
        gp1.update();
        gp2.update();
        robot.updateEverything();
        robot.updateLimelight();
    }

    private enum IntakeState {
        NEUTRAL,
        READY,
        TRANSFER,
        HOVER,
        OBS_ZONE
    }

    private Task createHoverTask() {
        Task hoverTask = new ParallelTask(
                new HorizontalSlidesTask(robot, 15.0,1.0,1500),
                new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                new SeriesTask(
                        new SleepTask(200),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER)
                )
        );
        return hoverTask;
    }
    private Task createIntakeRetryTask() {
        Task intakeRetryTask = new SeriesTask(
                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                        new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                ),
                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
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
                        new VerticalSlidesTask(robot, 0.0, 1.0, 750),
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER),
                        new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE)
                ),
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
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
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                        )
                )
        );
        return transferTask;
    }
    private Task createReadyTask() {
        Task readyTask = new ParallelTask(
                new HorizontalSlidesTask(robot, 0.0, 1.0, 1500),
                new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
                new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT)
        );
        return readyTask;
    }

    private Task createNeutralTask() {
        Task neutralTask = new ParallelTask(
                new HorizontalSlidesTask(robot, 0.0, 1.0, 1500),
                new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT)
        );
        return neutralTask;
    }

    private Task createAutoIntakeTask() {
        Coordinates samplePos = robot.getLimelightObjectPosition();
        SampleOrientation sampleOrientation = robot.getLimelightObjectOrientation();
        Coordinates invKin = InvKin.inverseKinematics(samplePos);

        Log.e("edbug sample cycles", samplePos.getX()+" "+samplePos.getY()+" "+samplePos.getAngle());
        Log.e("edbug sample cycles", sampleOrientation.toString());

        SeriesTask task = new SeriesTask();

        task.add(
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER_READY),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER_READY),
                        new SeriesTask(
                                new ParallelTask(
                                        new HorizontalSlidesTask(robot, invKin.getY(), 1.0, 1500),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                                        new IntakePivotTask(robot, sampleOrientation, invKin.getAngle()),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                                        new IntakeTurretTask(robot, invKin.getAngle()),
                                        new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                ),
                                new ParallelTask(
                                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                                ),
                                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                                new SleepTask(100)
                        )
                )
        );
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER)
                )
        );

        return task;
    }
    private Task createAutoChamberScoreTask() {
        robot.pinpoint.resetPosAndIMU();
        ParallelTask autoChamberScoreTask = new ParallelTask(
                new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                new SeriesTask(
                        new SleepTask(500),
                        new ParallelTask(
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = new Pose2d(AUTO_SPEC_X, AUTO_SPEC_Y, 0.0);
                                            return builder.
                                                    strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                    .build();
                                        }
                                ),
                                new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT, 1.0, 1500),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP),
                                new SeriesTask(
                                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP)
                                )
                        )
                )
        );
        return autoChamberScoreTask;

    }

    protected abstract boolean isRed();

}




