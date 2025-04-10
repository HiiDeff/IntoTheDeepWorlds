package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Axon Max+ servo has 0.14sec/60° -> 1/6/0.14*60 RPM
 * under 4.8 v and 355 degree max rotation. */
@Config
public class DeliveryTask extends TimedTask {
    public enum WhichDelivery {
        LEFT,
        RIGHT
    }


    private static final int RPM = 71;
    private static final int MAX_ROTATION_DEGREE = 355;


    private final RobotBase robot;
    private final Position position;
    private final WhichDelivery whichDelivery;
    private boolean started;

    public static void setDeliveryPosition(RobotBase robot,
                                       WhichDelivery placer, Position position) {
        getDeliveryServo(robot, placer).setPosition(robot.getDeliveryTargetPos(placer, position));
    }

    public DeliveryTask(RobotBase robot, WhichDelivery whichDelivery, Position position) {
        this.robot = robot;
        this.whichDelivery = whichDelivery;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = getDeliveryServo(robot, whichDelivery);
            double currPos = servo.getPosition();
            double targetPos = robot.getDeliveryTargetPos(whichDelivery, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos)/2);
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static Servo getDeliveryServo(RobotBase robot, WhichDelivery whichDelivery) {
        return whichDelivery == WhichDelivery.LEFT ? robot.deliveryLeft : robot.deliveryRight;
    }

    public enum Position {
        TRANSFER,
        FLAT,
        UP,
        CLIP,
        DELIVER,
        INIT
    }
}



