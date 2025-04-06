package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.SampleOrientation;

/**
 * Axon MINI+ servo has 0.110sec/60° -> 1/6/0.11*60 RPM
 * under 4.8 v and 355 degree max rotation.
 */
@Config
public class IntakePivotTask extends TimedTask {

    private static final int RPM = 91;
    private static final int MAX_ROTATION_DEGREE = 355;

    private RobotBase robot;
    private SampleOrientation position;
    private double exactPosition;
    private boolean started;

    public IntakePivotTask(RobotBase robot, SampleOrientation position) {
        this.robot = robot;
        this.position = position;
    }
    public IntakePivotTask(RobotBase robot, double angleDeg) {
        this.robot = robot;
        this.exactPosition = angleToPos(angleDeg);
    }
    public IntakePivotTask(RobotBase robot, SampleOrientation orientation, double offsetAngleDeg) {
        this.robot = robot;
        this.exactPosition = angleToPos(orientation.toAngleDeg()+offsetAngleDeg);
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
        //change angle to [0, 180), where 0 is right horizontal and 180 is left horizontal
        while(angleDeg<0) angleDeg+=180;
        while(angleDeg>=180) angleDeg-=180;
        double pos = robot.getIntakePivotPosition(SampleOrientation.VERTICAL);
        if(angleDeg>90){
            pos += (robot.getIntakePivotPosition(SampleOrientation.TILTED_LEFT)-robot.getIntakePivotPosition(SampleOrientation.VERTICAL))
                    *((angleDeg-90)/45.0);
        }else{
            pos += (robot.getIntakePivotPosition(SampleOrientation.TILTED_RIGHT)-robot.getIntakePivotPosition(SampleOrientation.VERTICAL))
                    *((angleDeg-90)/-45.0);
        }
        return pos;
    }
}



