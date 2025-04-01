package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;

@Config
public abstract class SampleAuto extends AutoBase {

    public static int verticalSlidesDelay = 500;
    public static int DELIVER_DRIVING_SPEED = 20;

    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();

        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos = getPreloadDeliverPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
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
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos;
                            if(spikeMarkNumber==1) pos = getSample1DeliverPos();
                            else if(spikeMarkNumber==2) pos = getSample2DeliverPos();
                            else pos = getSample3DeliverPos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
                )
        );
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose2d pos;
                            if(spikeMarkNumber==1) pos = getSample2IntakePos();
                            else if(spikeMarkNumber==2) pos = getSample3IntakePos();
                            else pos = getSubmersibleCyclePos();
                            return builder.strafeToLinearHeading(pos.position, pos.heading, new TranslationalVelConstraint(60)).build();
                        }
                )
        );

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
