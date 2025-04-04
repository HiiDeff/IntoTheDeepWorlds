package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Axon MINI+ servo has 0.110sec/60° -> 1/6/0.11*60 RPM
 * under 4.8 v and 355 degree max rotation.
 */
@Config
public class IntakePivotTask extends TimedTask {

    private static final int RPM = 91;
    private static final int MAX_ROTATION_DEGREE = 355;

    private RobotBase robot;
    private Position position;
    private double exactPosition;
    private boolean started;

    public IntakePivotTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }
    public IntakePivotTask(RobotBase robot, double angleDeg) {
        this.robot = robot;
        this.exactPosition = angleToPos(angleDeg);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.intakePivot;
            double currPos = servo.getPosition();
            double targetPos = position!=null ? robot.getIntakePivotPosition(position):exactPosition;
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private double angleToPos(double angleDeg) {
        //change angle to [-90, 90), where 0 is straight and left is negative
        while(angleDeg<-90) angleDeg+=180;
        while(angleDeg>=90) angleDeg-=180;
        double pos = robot.getIntakePivotPosition(Position.VERTICAL);
        if(angleDeg<0){
            pos += (robot.getIntakePivotPosition(Position.TILTED_LEFT)-robot.getIntakePivotPosition(Position.VERTICAL))
                    *(angleDeg/-45.0);
        }else{
            pos += (robot.getIntakePivotPosition(Position.TILTED_RIGHT)-robot.getIntakePivotPosition(Position.VERTICAL))
                    *(angleDeg/45.0);
        }
        return pos;
    }

    public enum Position {
        VERTICAL,
        VERTICAL_INVERSE,
        HORIZONTAL,
        TILTED_LEFT,
        TILTED_RIGHT
    }
}



