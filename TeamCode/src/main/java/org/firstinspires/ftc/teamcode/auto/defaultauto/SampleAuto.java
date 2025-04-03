package org.firstinspires.ftc.teamcode.auto.defaultauto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.conditions.SampleStateCondition;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.DecisionTask;
import org.firstinspires.ftc.teamcode.task.DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.DiffyTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
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

@Config
public abstract class SampleAuto extends AutoBase {

    public static int verticalSlidesDelay = 500;
    public static int DELIVER_DRIVING_SPEED = 20, DELIVER_CLAW_DELAY = 100, INTAKE_DELAY = 100;
    public static double DELIVERY_HEIGHT_INCH = 28.0;

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
                        new VerticalSlidesTask(robot, DELIVERY_HEIGHT_INCH, 1.0, 500),
                        new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.UP),
                        new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.UP),
                        new HorizontalSlidesTask(robot, 20.3,1.0,800),
                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                        new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                        new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                        new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL),
                        new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                )
        );
        task.add(
                new ParallelTask(
                        new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.DELIVER),
                        new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.DELIVER),
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
        Log.i("spike mark number", spikeMarkNumber+"");
        task.add(
                new ParallelTask(
                        new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.TRANSFER),
                        new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.TRANSFER),
                        new VerticalSlidesTask(robot, 0.0,1.0,500),
                        new SeriesTask(
                                new ParallelTask(
                                        new HorizontalSlidesTask(robot, 20.3,1.0,250),
                                        new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
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
                        new SeriesTask(
                                new IntakeArmTask(robot, IntakeArmTask.Position.SLIGHTLY_UP),
                                new ParallelTask(
                                        new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                                        new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER),
                                        new DecisionTask(
                                                new SampleStateCondition(robot),
                                                new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL_INVERSE),
                                                new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL)
                                        ),
                                        new HorizontalSlidesTask(robot, -0.0, 0.8, 1000)
                                )
                        )
                )
        );
        task.add(
                new ParallelTask(
                        new ParallelTask(
                                new HorizontalSlidesTask(robot, -2.0, 1.0, 700),
                                new SeriesTask(
                                        new ParallelTask(
                                                new DeliveryClawTask(robot, DeliveryClawTask.Position.CLOSE),
                                                new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                                        ),
                                        new ParallelTask(
                                                new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.UP),
                                                new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.UP),
                                                new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                                                new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                                                new IntakeWristTask(robot, IntakeWristTask.Position.TRANSFER),
                                                new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL),
                                                new VerticalSlidesTask(robot, DELIVERY_HEIGHT_INCH,1.0,500)
                                        )
                                )
                        )
                )
        );
        if(spikeMarkNumber<3) {
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new ParallelTask(
                                            new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.DELIVER),
                                            new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.DELIVER),
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
                            new HorizontalSlidesTask(robot, 20.3,1.0,900),
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                            new IntakeArmTask(robot, IntakeArmTask.Position.HOVER),
                            new IntakeWristTask(robot, IntakeWristTask.Position.HOVER),
                            new IntakePivotTask(robot, spikeMarkNumber==2? IntakePivotTask.Position.TILTED_RIGHT:IntakePivotTask.Position.VERTICAL),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
                    )
            );
        } else {
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new ParallelTask(
                                            new DiffyTask(robot, DiffyTask.WhichDiffy.LEFT, DiffyTask.Position.DELIVER),
                                            new DiffyTask(robot, DiffyTask.WhichDiffy.RIGHT, DiffyTask.Position.DELIVER),
                                            new SeriesTask(
                                                    new SleepTask(DELIVER_CLAW_DELAY),
                                                    new DeliveryClawTask(robot, DeliveryClawTask.Position.OPEN)
                                            )
                                    ),
                                    new RuntimeDrivingTask(
                                            robot,
                                            builder -> {
                                                Pose2d pos = getSubmersibleCyclePos();
                                                return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                                            }
                                    )
                            ),
                            new HorizontalSlidesTask(robot, 0.0,1.0,750),
                            new IntakeTurretTask(robot, IntakeTurretTask.Position.STRAIGHT),
                            new IntakeArmTask(robot, IntakeArmTask.Position.BACK),
                            new IntakeWristTask(robot, IntakeWristTask.Position.BACK),
                            new IntakePivotTask(robot, IntakePivotTask.Position.VERTICAL),
                            new IntakeClawTask(robot, IntakeClawTask.Position.OPEN)
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
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getCycleDeliverPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
                )
        );
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getSubmersibleCyclePos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
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
    protected abstract Pose2d getSample1IntakePos();
    protected abstract Pose2d getSample1DeliverPos();
    protected abstract Pose2d getSample2IntakePos();
    protected abstract Pose2d getSample2DeliverPos();
    protected abstract Pose2d getSample3IntakePos();
    protected abstract Pose2d getSample3DeliverPos();
    protected abstract Pose2d getSubmersibleCyclePos();
    protected abstract Pose2d getCycleDeliverPos();
}
