package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

public class VerticalSlidesPID extends PIDModel {

    private final RobotBase robot;
    private int targetPosition;

    public VerticalSlidesPID(RobotBase robot, PIDCoefficients pidCoefficients) {
        super(pidCoefficients);
        this.robot = robot;
    }

    public void setTargetPosition(int targetPositionTicks) {
        targetPosition = targetPositionTicks;
    }

    @Override
    public void cancel() {
        //what to do if cancel? probably just holding power but don't use cancel()
    }

    @Override
    public double getError() {
        return targetPosition - robot.getVerticalSlidesHeightTicks(); //return error of position
    }

    @Override
    protected double getStopError() {
        return 5;
    } //minimum error to stop adjusting

    @Override
    protected double getStopSpeed() {
        return 1; //0.001 ticks/ms = 1 ticks/sec
    } //minimum speed to stop adjusting (ticks per ms)
}
