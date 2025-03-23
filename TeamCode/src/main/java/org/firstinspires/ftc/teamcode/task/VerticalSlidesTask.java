package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class VerticalSlidesTask extends TimedTask{

    private RobotBase robot;
    private final double targetHeight;
    private final double power;
    private boolean started = false;

    public VerticalSlidesTask(RobotBase robot, double height) {
        this(robot, height, 0.9);
    }

    public VerticalSlidesTask(RobotBase robot, double height, double power) {
        this(robot, height, power, height < 1 ? 1500 : 1000);
    }

    public VerticalSlidesTask(RobotBase robot, double height, double power,
                              int finishTimeMillis) {
        this.robot = robot;
        this.targetHeight = height;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setVerticalSlidesHeight(targetHeight);
        }
        if (targetHeight < 0.001 &&
                Math.abs(robot.getVerticalSlidesHeightInch() - targetHeight) < 0.1) {
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        if (targetHeight < 1) {
            robot.setVerticalSlidePowerWithoutEncoder(0.0);
        }
    }
}
