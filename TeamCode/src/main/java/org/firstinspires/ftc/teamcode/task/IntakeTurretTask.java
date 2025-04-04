package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Axon Max+ servo has 0.14sec/60° -> 1/6/0.14*60 RPM
 * under 4.8 v and 355 degree max rotation. */
@Config
public class IntakeTurretTask extends TimedTask {

    private static final int RPM = 71;
    private static final int MAX_ROTATION_DEGREE = 355;

    private RobotBase robot;
    private Position position;
    private double exactPosition;
    private boolean started;

    public IntakeTurretTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }
    public IntakeTurretTask(RobotBase robot, double angleDeg) {
        this.robot = robot;
        this.exactPosition = angleToPos(angleDeg);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.turret;
            double currPos = servo.getPosition();
            double targetPos = position!=null? robot.getTurretPosition(position):exactPosition;
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos)/2);
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private double angleToPos(double angleDeg) {
        //change angle to [-90, 90), where 0 is straight and left is negative
        while(angleDeg<-90) angleDeg+=180;
        while(angleDeg>=90) angleDeg-=180;
        double pos = robot.getTurretPosition(Position.STRAIGHT);
        if(angleDeg<0){
            pos += (robot.getTurretPosition(Position.LEFT_45_DEG)-robot.getTurretPosition(Position.STRAIGHT))
                    *(angleDeg/-45.0);
        }else{
            pos += (robot.getTurretPosition(Position.RIGHT_45_DEG)-robot.getTurretPosition(Position.STRAIGHT))
                    *(angleDeg/45.0);
        }
        return pos;
    }

    public enum Position {
        STRAIGHT,
        DROP_OFF,
        LEFT_45_DEG,
        RIGHT_45_DEG
    }
}



