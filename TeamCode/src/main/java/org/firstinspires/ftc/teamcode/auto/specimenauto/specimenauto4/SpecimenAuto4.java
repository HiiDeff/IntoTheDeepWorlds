package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;

import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;

@Config
public abstract class SpecimenAuto4 extends SpecimenAuto4Base {

    public static double PRELOAD_BEGIN_END_VELO = 0.0;
    public static double CHAMBER_DELIVER_HEIGHT = 19.0, CHAMBER_DELIVER_DIST_DOWN = 8.5, BEG_END_VEL_FOR_FAST_PATHS = 20.0;
    public static int DELIVER_DELAY = 0, DELIVERY_CLAW_OPEN_DELAY = 300, DELIVERY_CLAW_CLOSE_DELAY = 100,
            FIRST_SAMPLE_EXTEND_DELAY = 500, INTAKE_DELAY_SPIKE_MARKS = 000, INTAKE_DELAY = 200, SAMPLE_INTAKE_TO_OBS_POS_DELAY = 500,
            SPIKE_MARK_INTAKE_TANGENT_DEG = -80;

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
                                PRELOAD_BEGIN_END_VELO
                        ),
                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT, 1.0, 1000),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP)
                )
        );
        task.add(
                new ParallelTask(
                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0,750),
                        new SeriesTask(
                                new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                        )
                )
        );
        return task;
    }

    @Override
    protected Task createSpikeMarkTask() {
        state = AutoState.SPIKE_MARKS;
        SeriesTask task = new SeriesTask();

        task.add(new ParallelTask(
                new HorizontalSlidesTask(robot, -1.0, 1.0, 1000),
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSpikeMark1IntakePos();
                            if(autoStates.getSpikeMarkNumber()==2) pos = getSpikeMark2IntakePos();
                            else if(autoStates.getSpikeMarkNumber()==3) pos = getSpikeMark3IntakePos();
                            return builder.
                                    strafeToLinearHeading(pos.position, pos.heading)
                                    .build();
                        }
                ),
                new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT),
                new VerticalSlidesTask(robot, 0.0, 1.0, 1000),
                new SeriesTask(
                        new SleepTask(autoStates.getSpikeMarkNumber()==1?750:0),
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                                new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                                new IntakePivotTask(robot, SampleOrientation.TILTED_LEFT)
                        )
                )
        ));
        task.add(
                new SeriesTask(
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.DOWN),
                                new IntakeWristTask(robot, IntakeWristTask.Position.DOWN)
                        ),
                        new IntakeClawTask(robot, IntakeClawTask.Position.CLOSE),
                        new ParallelTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                                new IntakeWristTask(robot, IntakeWristTask.Position.SLIGHTLY_UP),
                                new SeriesTask(
                                        new SleepTask(autoStates.getSpikeMarkNumber()==1?0:100),
                                        new HorizontalSlidesTask(robot, 17.0, 1.0,600)
                                ),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getSpikeMark1DeliverPos();
                                            if(autoStates.getSpikeMarkNumber()==2) pos = getSpikeMark2DeliverPos();
                                            else if(autoStates.getSpikeMarkNumber()==3) pos = getSpikeMark3DeliverPos();
                                            if(autoStates.getSpikeMarkNumber()==1||autoStates.getSpikeMarkNumber()==2) {
                                                return builder.turnTo(pos.heading, new TurnConstraints(4*Math.PI, -4*Math.PI, 4*Math.PI))
                                                        .build();
                                            }
                                            else {
                                                return builder
                                                        .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                        .build();
                                            }
                                        }
                                )
                        )
                )
        );
        task.add(new ParallelTask(
                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN),
                new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                new SeriesTask(
                        new SleepTask(300),
                        new HorizontalSlidesTask(robot, 0.0, 1.0, 1500)
                )
        ));
        if(autoStates.getSpikeMarkNumber()==AA_NUM_OF_SPIKE_MARKS) {
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(
                                    robot,
                                    builder -> {
                                        Pose2d pos = getCycleIntakePos();
                                        return builder.
                                                strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(80))
                                                .build();
                                    },
                                    0.0
                            ),
                            new HorizontalSlidesTask(robot, 0.0, 1.0, 750),
                            new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                            new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                            new IntakePivotTask(robot, SampleOrientation.VERTICAL)
                    )
            );
        }
        return task;
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        SeriesTask task = new SeriesTask();

        task.add(
                new ParallelTask(
                        new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                        new SeriesTask(
                                new SleepTask(DELIVERY_CLAW_CLOSE_DELAY),
                                new ParallelTask(
                                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT,1.0, 1000),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.CLIP),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.CLIP),
                                        new RuntimeDrivingTask(
                                                robot,
                                                builder -> {
                                                    Pose2d pos = getCycleDeliverPos();
                                                    return builder
                                                            .strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(90))
                                                            .build();
                                                }
                                        )
                                )
                        )
                )
        );

        task.add(
                new ParallelTask(
                        new SeriesTask(
                                new ParallelTask(
                                        new VerticalSlidesTask(robot, CHAMBER_DELIVER_HEIGHT-CHAMBER_DELIVER_DIST_DOWN, 1.0,750),
                                        new SeriesTask(
                                                new SleepTask(DELIVERY_CLAW_OPEN_DELAY),
                                                new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                        )
                                ),
                                new ParallelTask(
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.LEFT, DeliveryTask.Position.FLAT),
                                        new DeliveryTask(robot, DeliveryTask.WhichDelivery.RIGHT, DeliveryTask.Position.FLAT)
                                ),
                                new VerticalSlidesTask(robot, 0.0,1.0,1000)
                        ),
                        new SeriesTask(
                                new SleepTask(DELIVER_DELAY),
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose2d pos = getCycleIntakePos();
                                            return builder
                                                    .strafeToLinearHeading(pos.position, pos.heading)
                                                    .build();
                                        }

                                )
                        )
                )
        );

        return task;
    }

    @Override
    protected Task createFinishTask() {
        return null;
    }

    protected abstract Pose2d getPreloadDeliverPos();
    protected abstract Pose2d getCycleIntakePos();
    protected abstract Pose2d getCycleDeliverPos();
    protected abstract Pose2d getSpikeMark1IntakePos();
    protected abstract Pose2d getSpikeMark1DeliverPos();
    protected abstract Pose2d getSpikeMark2IntakePos();
    protected abstract Pose2d getSpikeMark2DeliverPos();
    protected abstract Pose2d getSpikeMark3IntakePos();
    protected abstract Pose2d getSpikeMark3DeliverPos();

}
