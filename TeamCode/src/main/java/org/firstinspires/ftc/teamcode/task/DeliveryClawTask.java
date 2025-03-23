package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

public class DeliveryClawTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final RobotBase robot;
    private boolean started;
    private final Position position;

    public DeliveryClawTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.deliveryClaw.getPosition();
            double targetPos = robot.getDeliveryClawPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.deliveryClaw.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        OPEN,
        CLOSE
    }
}
