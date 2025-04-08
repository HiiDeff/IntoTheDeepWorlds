package org.firstinspires.ftc.teamcode.task;


import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;

public class HorizontalSlidesWaitForTransferTask extends TimedTask {

    private RobotBase robot;
    private final double distInInches;

    public HorizontalSlidesWaitForTransferTask(RobotBase robot, double distInInches,
                                int finishTimeMillis) {
        this.robot = robot;
        this.distInInches = distInInches;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        return Math.abs(robot.getHorizontalSlideDistInch() - distInInches) < 0.5;
    }

    @Override
    public void cancel() {
        //do nothing
    }
}
