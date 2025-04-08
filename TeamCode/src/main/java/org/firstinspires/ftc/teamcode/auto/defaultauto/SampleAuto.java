package org.firstinspires.ftc.teamcode.auto.defaultauto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.task.DecisionTask;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesWaitForTransferTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesWithoutCancelTask;
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
import org.firstinspires.ftc.teamcode.task.UnboundedHorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;
import org.firstinspires.ftc.teamcode.common.InvKin;


@Config
public abstract class SampleAuto extends AutoBase {

    public static int verticalSlidesDelay = 500, DELIVER_EXTEND_MIN_TIME = 1500;
    public static int DELIVER_DRIVING_SPEED = 20, DELIVER_CLAW_DELAY = 100, INTAKE_DELAY = 100;
    public static double DELIVERY_HEIGHT_INCH = 34.0;
    public static int BUCKET_TO_SUB_START_TANGENT_DEG = -30, BUCKET_TO_SUB_END_TANGENT_DEG = -60;

    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos = getPreloadDeliverPos();
                                    return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                                }
                        ),
                        new VerticalSlidesTask(robot, DELIVERY_HEIGHT_INCH, 1.0, DELIVER_EXTEND_MIN_TIME),
                        new HorizontalSlidesTask(robot, 21.3,1.0,800),
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER)
                )
        );
        task.add(
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                        new SeriesTask(
                                new SleepTask(DELIVER_CLAW_DELAY),
                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                        )
                )
        );
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSample1IntakePos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
                )
        );

        return task;
    }

    @Override
    protected Task createSpikeMarkTask() {
        state = AutoState.SPIKE_MARKS;
        SeriesTask task = new SeriesTask();
        int spikeMarkNumber = autoStates.getSpikeMarkNumber();
        task.add(
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER),
                        new VerticalSlidesTask(robot, 0.0,1.0,1000),
                        new SeriesTask(
                                new ParallelTask(
                                        new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                                ),
                                new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                                new SleepTask(INTAKE_DELAY)
                        )
                )
        );
        //transfer
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos;
                                    if(spikeMarkNumber==1) pos = getSample1DeliverPos();
                                    else if(spikeMarkNumber==2) pos = getSample2DeliverPos();
                                    else pos = getSample3DeliverPos();
                                    return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                                }
                        ),
                        createTransferTask()
                )
        );

        if(spikeMarkNumber<3) {
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new ParallelTask(
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                                            new SeriesTask(
                                                    new SleepTask(DELIVER_CLAW_DELAY),
                                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                            )
                                    ),
                                    new RuntimeDrivingTask(
                                            robot,
                                            builder -> {
                                                Pose2d pos = getSubmersibleCyclePos();
                                                if(spikeMarkNumber==1) pos = getSample2IntakePos();
                                                else if(spikeMarkNumber==2) pos = getSample3IntakePos();
                                                return builder.strafeToLinearHeading(pos.position, pos.heading).build();
                                            }
                                    )
                            ),
                            new HorizontalSlidesTask(robot, 21,1.0,1000),
                            new IntakeTurretTask(robot, spikeMarkNumber==2? IntakeTurretTask.Position.LEFT_45_DEG : IntakeTurretTask.Position.STRAIGHT),
                            new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                            new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                            new IntakePivotTask(robot, spikeMarkNumber==2? SampleOrientation.TILTED_RIGHT:SampleOrientation.VERTICAL),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                    )
            );
        } else {
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new ParallelTask(
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                                            new SeriesTask(
                                                    new SleepTask(DELIVER_CLAW_DELAY),
                                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                            )
                                    ),
                                    new ParallelTask(
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                                            new VerticalSlidesTask(robot, 0.0, 1.0, 1000)
                                    )
                            ),
                            new SeriesTask(
                                    new SleepTask(500),
                                    new RuntimeDrivingTask(
                                            robot,
                                            builder -> {
                                                Pose2d pos = getSubmersibleCyclePos();
                                                return builder
                                                        .setTangent(Math.toRadians(BUCKET_TO_SUB_START_TANGENT_DEG))
                                                        .splineToSplineHeading(pos, Math.toRadians(BUCKET_TO_SUB_END_TANGENT_DEG), new TranslationalVelConstraint(60))
                                                        .build();
                                            }
                                    )
                            ),
                            new HorizontalSlidesTask(robot, 0.0,1.0,750),
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                            new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                            new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                            new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                    )
            );
        }


        return task;
    }
    @Override
    protected Task createTurnToSampleTask() {
        state = AutoState.TURN_TO_SAMPLE;
        Coordinates samplePos = robot.getLimelightObjectPosition();
        double angleToSample = robot.getAngleToSample(samplePos);
        Log.i("edbug angle to sample", angleToSample+"");
        SeriesTask task = new SeriesTask();
        task.add(
                new RuntimeDrivingTask(
                    robot,
                    builder -> {
                        double currHeading = robot.getPositionRR().heading.toDouble();
                        return builder
                                .turnTo(currHeading-angleToSample)
                                .build();
                    }
            )
        );
        task.add(new SleepTask(500));
        return task;
    }
    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        Coordinates samplePos = robot.getLimelightObjectPosition();
        SampleOrientation sampleOrientation = robot.getLimelightObjectOrientation();
        Coordinates invKin = InvKin.inverseKinematics(new Coordinates(samplePos.getX(), samplePos.getY(), samplePos.getAngle()));

        Log.e("edbug sample cycles", samplePos.getX()+" "+samplePos.getY()+" "+samplePos.getAngle());
        Log.e("edbug sample cycles", sampleOrientation.toString());

        SeriesTask task = new SeriesTask();

        task.add(
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER),
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
                                new SleepTask(INTAKE_DELAY)
                        )
                )
        );

//        task.add(
//                new ParallelTask(
//                        new RuntimeDrivingTask(
//                                robot,
//                                builder -> {
//                                    Pose2d currPos = robot.getPositionRR();
////                                    double theta = currPos.heading.toDouble() - (-Math.PI/2);
////                                    double yComponent = -Math.sin(theta)*samplePos.getX();
////                                    double xComponent = currPos.position.x - Math.cos(theta)*samplePos.getX();
//                                    Pose2d newPos = new Pose2d(currPos.position.x-samplePos.getX(), currPos.position.y, currPos.heading.toDouble());
//                                    return builder
//                                            .strafeToLinearHeading(newPos.position, newPos.heading, new TranslationalVelConstraint(70))
//                                            .build();
//                                }
//                        ),
//                        new IntakePivotTask(robot, sampleOrientation),
//                        new HorizontalSlidesTask(robot, slideExtendLengthInch,1.0,750),
//                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
//                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
//                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
//                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
//                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.TRANSFER),
//                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.TRANSFER)
//
//                )
//        );
//        task.add(
//                new SeriesTask(
//                        new ParallelTask(
//                                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
//                                new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
//                        ),
//                        new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
//                        new SleepTask(INTAKE_DELAY)
//                )
//        );
//

        task.add(
                new ParallelTask(
                        new SeriesTask(
                                new SleepTask(500),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getCycleDeliverPos();
                                            return builder
                                                    .setTangent(Math.toRadians(BUCKET_TO_SUB_END_TANGENT_DEG+180))
                                                    .splineToSplineHeading(pos, Math.toRadians(BUCKET_TO_SUB_START_TANGENT_DEG+180), new TranslationalVelConstraint(80))
                                                    .build();
                                        }
                                )
                        ),
                        createTransferTask()
                )
        );
        task.add(
                new ParallelTask(
                        new SeriesTask(
                                new ParallelTask(
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.DELIVER),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.DELIVER),
                                        new SeriesTask(
                                                new SleepTask(DELIVER_CLAW_DELAY),
                                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                        )
                                ),
                                new ParallelTask(
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                                        new VerticalSlidesTask(robot, 0.0, 1.0, 1000)
                                )
                        ),
                        new SeriesTask(
                                new SleepTask(500),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getSubmersibleCyclePos();
                                            return builder
                                                    .setTangent(Math.toRadians(BUCKET_TO_SUB_START_TANGENT_DEG))
                                                    .splineToSplineHeading(pos, Math.toRadians(BUCKET_TO_SUB_END_TANGENT_DEG), new TranslationalVelConstraint(80))
                                                    .build();
                                        }
                                )
                        ),
                        new HorizontalSlidesTask(robot, 0.0,1.0,750),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                        new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                        new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                        new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                )
        );

        return task;
    }
    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();

        return task;
    }


    private Task createTransferTask() {
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                        new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
                        new DecisionTask(
                                new SampleStateCondition(robot),
                                new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                                new IntakePivotTask(robot, SampleOrientation.VERTICAL_INVERSE)
                        )
                )
        );
        task.add(
                new ParallelTask(
                        new HorizontalSlidesWithoutCancelTask(robot, -0.5, 1.0, 1500),
                        new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                        new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER)
                )
        );
        task.add(
                new HorizontalSlidesWaitForTransferTask(robot, 0.0,1500)
        );
        task.add(
                new ParallelTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                        new SeriesTask(
                                new SleepTask(100),
                                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                        )
                )
        );
        task.add(
                new ParallelTask(
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.UP),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.UP),
                        new HorizontalSlidesTask(robot, 0.0, 1.0, 500),
                        new VerticalSlidesTask(robot, DELIVERY_HEIGHT_INCH,1.0,DELIVER_EXTEND_MIN_TIME)
                )
        );
        return task;
    }

    protected abstract Pose2d getPreloadDeliverPos();
    protected abstract Pose2d getSample1IntakePos();
    protected abstract Pose2d getSample1DeliverPos();
    protected abstract Pose2d getSample2IntakePos();
    protected abstract Pose2d getSample2DeliverPos();
    protected abstract Pose2d getSample3IntakePos();
    protected abstract Pose2d getSample3DeliverPos();
    protected abstract Pose2d getSubmersibleCyclePos();
    protected abstract Pose2d getCycleDeliverPos();
}
