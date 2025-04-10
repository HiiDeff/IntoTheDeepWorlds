package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.TrajectoryActioner;
import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class RuntimeDrivingTask implements Task {

    private final RobotBase robot;
    private final TrajectoryActioner actioner;
    private boolean started = false, endEarly = false;
    private double beginEndVelo = 0.0, endEarlyAbsDist = 0.0;
    private Action action;

    public RuntimeDrivingTask(RobotBase robot, TrajectoryActioner actioner) {
        this.robot = robot;
        this.actioner = actioner;
    }
    public RuntimeDrivingTask(RobotBase robot, TrajectoryActioner actioner, double beginEndVelo) {
        this.robot = robot;
        this.actioner = actioner;
        this.beginEndVelo = beginEndVelo;
    }

    @Override
    public boolean perform() {
        if (!started) {
            started = true;
            action = actioner.trajectory2Action(robot.actionBuilder(robot.pose, beginEndVelo));
        }
        return !action.run(new TelemetryPacket());
    }
}
