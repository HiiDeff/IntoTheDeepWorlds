package org.firstinspires.ftc.teamcode.task;


import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;

public class HorizontalSlidesTask extends TimedTask {

    private RobotBase robot;
    private final double distInInches;
    private final double power;
    private boolean started = false;

    public HorizontalSlidesTask(RobotBase robot, double distInInches, double power,
                                  int finishTimeMillis) {
        this.robot = robot;
        this.distInInches = distInInches;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setHorizontalSlideDist(this.distInInches, power);
        }
        if (Math.abs(robot.getHorizontalSlideDistInch() - distInInches) < 0.5)
        {
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        robot.setHorizontalSlidesPower(0.0);
    }
}
