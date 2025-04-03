package org.firstinspires.ftc.teamcode.auto.conditions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.Condition;
import org.firstinspires.ftc.teamcode.drive.RobotBase;

@Config
public class SampleStateCondition implements Condition {

    private final RobotBase robot;

    public SampleStateCondition(RobotBase robot) {
        this.robot = robot;
    }
    @Override
    public boolean shouldContinue() {
        return robot.isSampleDetected();
    }
}
