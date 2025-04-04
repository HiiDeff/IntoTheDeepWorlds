package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * torque servo has 40 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class HangTask extends TimedTask {
    public enum WhichHang {
        LEFT,
        RIGHT
    }

    private static final int RPM = 40;
    private static final int MAX_ROTATION_DEGREE = 300;


    private final RobotBase robot;
    private final Position position;
    private final WhichHang hang;
    private boolean started;

    public static void setHangPosition(RobotBase robot,
                                       WhichHang placer, Position position) {
        getHangServo(robot, placer).setPosition(robot.getHangTargetPos(placer, position));
    }

    public HangTask(RobotBase robot, WhichHang hang, Position position) {
        this.robot = robot;
        this.hang = hang;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = getHangServo(robot, hang);
            double currPos = servo.getPosition();
            double targetPos = robot.getHangTargetPos(hang, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static Servo getHangServo(RobotBase robot, WhichHang hang) {
        return hang == WhichHang.LEFT ? robot.leftHang : robot.rightHang;
    }

    public enum Position {
        INIT,
        ENGAGED
    }
}



