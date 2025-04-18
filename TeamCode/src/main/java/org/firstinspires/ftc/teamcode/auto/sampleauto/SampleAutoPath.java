package org.firstinspires.ftc.teamcode.auto.sampleauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
@Autonomous(name = "SAMPLE Auto")
public class SampleAutoPath extends SampleAuto {

    public static Pose2d
            preloadDeliverPos = new Pose2d(7.1656, 21.8393, -0.3651),
            sample1IntakePos = new Pose2d(7.1656, 21.8393, -0.3651),
            sample1DeliverPos = new Pose2d(5.8525, 23.1857, -0.0849),
            sample2IntakePos = new Pose2d(5.1525, 21.5, -0.0449),
            sample2DeliverPos = new Pose2d(5.1525, 23.1857, -0.20),
            sample3IntakePos = new Pose2d(7.7623,21,0.19),
            sample3DeliverPos = new Pose2d(5.3, 21.7, -0.4),
            submersibleCyclePos = new Pose2d(58.5983, -13.5819, -Math.PI/2),
            cycleDeliverPos = new Pose2d(2.3, 20.2, -0.5);

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadDeliverPos() {
        return preloadDeliverPos;
    }

    @Override
    protected Pose2d getSample1IntakePos() {
        return sample1IntakePos;
    }

    @Override
    protected Pose2d getSample1DeliverPos() {
        return sample1DeliverPos;
    }

    @Override
    protected Pose2d getSample2IntakePos() {
        return sample2IntakePos;
    }

    @Override
    protected Pose2d getSample2DeliverPos() {
        return sample2DeliverPos;
    }

    @Override
    protected Pose2d getSample3IntakePos() {
        return sample3IntakePos;
    }

    @Override
    protected Pose2d getSample3DeliverPos() {
        return sample3DeliverPos;
    }

    @Override
    protected Pose2d getSubmersibleCyclePos() {
        return submersibleCyclePos;
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return cycleDeliverPos;
    }
}
