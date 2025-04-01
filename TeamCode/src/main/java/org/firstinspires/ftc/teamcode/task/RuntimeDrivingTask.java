package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.TrajectoryActioner;
import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class RuntimeDrivingTask implements Task {

    private final RobotBase robot;
    private final TrajectoryActioner actioner;
    private boolean started = false;
    private Action action;

    public RuntimeDrivingTask(RobotBase robot, TrajectoryActioner actioner) {
        this.robot = robot;
        this.actioner = actioner;
    }

    @Override
    public boolean perform() {
        if (!started) {
            started = true;
            action = actioner.trajectory2Action(robot.actionBuilder(robot.pose));
        }
        return !action.run(new TelemetryPacket());
    }
}
