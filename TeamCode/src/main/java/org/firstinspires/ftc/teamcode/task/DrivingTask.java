package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public class DrivingTask implements Task {

    private final Action action;
    private boolean started = false;

    public DrivingTask(TrajectoryActionBuilder trajectory) {
        this.action = trajectory.build();
    }

    @Override
    public boolean perform() {
        return !action.run(new TelemetryPacket());
    }
}
