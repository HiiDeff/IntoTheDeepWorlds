package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;

import org.firstinspires.ftc.teamcode.common.InvKin;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
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
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;

@Config
public abstract class SpecimenAuto extends SpecimenAutoBase {

    public static double CHAMBER_DELIVER_HEIGHT = 19.4, CHAMBER_DELIVER_DIST_DOWN = 7.5, BEG_END_VEL_FOR_FAST_PATHS = 10.0;
    public static int DELIVER_DELAY = 0, DELIVERY_CLAW_OPEN_DELAY = 200, DELIVERY_CLAW_CLOSE_DELAY = 100,
            FIRST_SAMPLE_EXTEND_DELAY = 1500, INTAKE_DELAY_SPIKE_MARKS = 000, INTAKE_DELAY = 200;


    //FOR FIRST CYCLE, MAKE IT TURN AND PICK ONE UP
    //FOR THE REST, STAY CLOSE TO THE OBS ZONE (WITH THE TURRET UP AND EVERYTHING)

    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                        new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos = getPreloadDeliverPos();
                                    return builder
                                            .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                            .build();
                                },
                                BEG_END_VEL_FOR_FAST_PATHS
                        ),
                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT, 1.0, 1000),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                )
        );
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos = getSpikeMark1IntakePos();
                                    return builder
                                            .strafeToLinearHeading(pos.position, pos.heading)
                                            .build();
                                }
                        ),
                        new SeriesTask(
                                new SleepTask(FIRST_SAMPLE_EXTEND_DELAY),
                                new ParallelTask(
                                        new HorizontalSlidesTask(robot, 20.0, 1.0,1200),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                                        new IntakePivotTask(robot, SampleOrientation.TILTED_LEFT)
                                )
                        ),
                        new SeriesTask(
                                new SleepTask(DELIVER_DELAY),
                                new ParallelTask(
                                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0, 500),
                                        new SeriesTask(
                                                new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                        )
                                ),
                                new ParallelTask(
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT)
                                ),
                                new VerticalSlidesTask(robot, 0.0, 1.0, 500)
                        )
                )
        );
        return task;
    }
    @Override
    protected Task createSpikeMarkTask() {
        state = AutoState.SPIKE_MARKS;
        int spikeMarkNo = autoStates.getSpikeMarkNumber(); //to deliver # 1, 2, or 3
        SeriesTask task = new SeriesTask();
        task.add(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                                new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                        ),
                        new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                        new SleepTask(INTAKE_DELAY_SPIKE_MARKS)
                )
        );
        task.add(
                new ParallelTask(
                        new HorizontalSlidesTask(robot, 12.0,1.0,1000),
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos;
                                    if(spikeMarkNo==1) pos = getSpikeMark1DeliverPos();
                                    else if(spikeMarkNo==2) pos = getSpikeMark2DeliverPos();
                                    else pos = getSpikeMark3DeliverPos();
                                    return builder
                                            .turnTo(pos.heading, new TurnConstraints(4*Math.PI,-4*Math.PI,2*Math.PI))
                                            .build();
                                },
                                10
                        )
                )
        );
        task.add(new IntakeClawTask(robot, IntakeClawTask.Position.OPEN));
        if(spikeMarkNo<AA_NUM_OF_SPIKE_MARKS) {
            task.add(
                    new ParallelTask(
                            new HorizontalSlidesTask(robot, 20.0, 1.0, 1200),
                            new RuntimeDrivingTask(
                                    robot,
                                    builder -> {
                                        Pose2d pos = getSpikeMark2IntakePos();
                                        if (spikeMarkNo == 2) pos = getSpikeMark3IntakePos();
                                        return builder
                                                .strafeToLinearHeading(pos.position, pos.heading)
                                                .build();
                                    }
                            )
                    )
            );
        } else {
            task.add(
                    new ParallelTask(
                            new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                            new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
                            new HorizontalSlidesTask(robot, 0.0,1.0,1200),
                            new VerticalSlidesTask(robot, 0.0, 1.0,500),
                            new RuntimeDrivingTask(
                                    robot,
                                    builder -> {
                                        Pose2d pos = getCycleIntakePos();
                                        return builder
                                                .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                .build();
                                    },
                                    BEG_END_VEL_FOR_FAST_PATHS
                            )
                    )
            );
            task.add(
                    new ParallelTask(
                            new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                            new SeriesTask(
                                    new SleepTask(DELIVERY_CLAW_CLOSE_DELAY),
                                    new ParallelTask(
                                            new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT,1.0, 1000),
                                            new RuntimeDrivingTask(
                                                    robot,
                                                    builder -> {
                                                        Pose2d pos = getCycleDeliverPos();
                                                        return builder
                                                                .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                                .build();
                                                    },
                                                    0.0
                                            ),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                                            new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                                    )
                            )

                    )
            );
        }
        return task;
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        SeriesTask task = new SeriesTask();
        Coordinates samplePos = robot.getLimelightObjectPosition();
        SampleOrientation sampleOrientation = robot.getLimelightObjectOrientation();
        Coordinates invKin = InvKin.inverseKinematics(samplePos);

        if(samplePos.getY()!=-1 && autoStates.getIntakenSampleCnt() < AA_NUM_TO_INTAKE) {
            autoStates.setIntakenSampleCnt(autoStates.getIntakenSampleCnt()+1);
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new ParallelTask(
                                            new HorizontalSlidesTask(robot, invKin.getY(), 1.0, 1500),
                                            new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                                            new IntakePivotTask(robot, sampleOrientation, invKin.getAngle()),
                                            new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                                            new IntakeTurretTask(robot, invKin.getAngle())
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
            task.add(
                    new ParallelTask(
                            new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0, 500),
                            new SeriesTask(
                                    new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                            ),
                            new SeriesTask(
                                    new ParallelTask(
                                            new IntakeArmTask(robot, IntakeArmTask.Position.OBS_ZONE),
                                            new IntakeWristTask(robot, IntakeWristTask.Position.OBS_ZONE),
                                            new IntakePivotTask(robot, SampleOrientation.HORIZONTAL)
                                    ),
                                    new IntakeTurretTask(robot, IntakeTurretTask.Position.OBS_ZONE)
                            )
                    )
            );
        } else {
            task.add(
                    new ParallelTask(
                            new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0, 500),
                            new SeriesTask(
                                    new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                            )
                    )
            );
        }

        task.add(
                new ParallelTask(
                        new HorizontalSlidesTask(robot, 0.0, 1.0, 1500),
                        new SeriesTask(
                                new ParallelTask(
                                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0, 500),
                                        new SeriesTask(
                                                new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                        )
                                ),
                                new ParallelTask(
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT),
                                        new VerticalSlidesTask(robot, 0.0, 1.0, 500)
                                )
                        ),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose2d pos = getCycleIntakePos();
                                    return builder
                                            .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                            .build();
                                },
                                BEG_END_VEL_FOR_FAST_PATHS
                        )
                )
        );
        task.add(new IntakeClawTask(robot, IntakeClawTask.Position.OPEN));
        task.add(
                new ParallelTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                        new SeriesTask(
                                new SleepTask(DELIVERY_CLAW_CLOSE_DELAY),
                                new ParallelTask(
                                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT,1.0, 1000),
                                        new RuntimeDrivingTask(
                                                robot,
                                                builder -> {
                                                    Pose2d pos = getCycleDeliverPos();
                                                    return builder
                                                            .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                            .build();
                                                },
                                                BEG_END_VEL_FOR_FAST_PATHS
                                        ),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                                )
                        ),
                        new SeriesTask(
                                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                                new ParallelTask(
                                        new IntakePivotTask(robot, SampleOrientation.VERTICAL),
                                        new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP)
                                )
                        )

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

    protected abstract Pose2d getPreloadDeliverPos();
    protected abstract Pose2d getSpikeMark1IntakePos();
    protected abstract Pose2d getSpikeMark1DeliverPos();
    protected abstract Pose2d getSpikeMark2IntakePos();
    protected abstract Pose2d getSpikeMark2DeliverPos();
    protected abstract Pose2d getSpikeMark3IntakePos();
    protected abstract Pose2d getSpikeMark3DeliverPos();
    protected abstract Pose2d getCycleIntakePos();
    protected abstract Pose2d getCycleDeliverPos();

}
