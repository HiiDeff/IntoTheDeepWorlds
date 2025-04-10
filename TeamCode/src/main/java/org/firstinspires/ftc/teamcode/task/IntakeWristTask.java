package org.firstinspires.ftc.teamcode.task;

import android.util.Log;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Axon Max+ servo has 0.14sec/60° -> 1/6/0.14*60 RPM
 * under 4.8 v and 355 degree max rotation. */
public class IntakeWristTask extends TimedTask {

    private static final int RPM = 71;
    private static final int MAX_ROTATION_DEGREE = 355;

    private final RobotBase robot;
    private boolean started;
    private final Position position;

    public IntakeWristTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.intakeWrist.getPosition();
            double targetPos = robot.getIntakeWristPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos)/2);
//            Log.i("posDiff",""+(targetPos-currPos));
            robot.intakeWrist.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        TRANSFER,
        HOVER,
        BACK,
        DOWN,
        SLIGHTLY_UP,
        OBS_ZONE
    }
}
