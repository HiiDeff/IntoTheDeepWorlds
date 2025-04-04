package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;

@Config
@Autonomous(name = "SAMPLE Auto")
public class SampleAutoPath extends SampleAuto {

    public static Pose2d
            preloadDeliverPos = new Pose2d(2.2589, 22.2425, -0.3637),
            sample1IntakePos = new Pose2d(5.6732, 17.8836, -0.2982),
            sample1DeliverPos = new Pose2d(1.6785, 21.671, -0.452),
            sample2IntakePos = new Pose2d(5.4798, 17.2887, -0.02),
            sample2DeliverPos = new Pose2d(0.1476, 22.103, -0.7399),
            sample3IntakePos = new Pose2d(13.2138, 15.2473, 0.7038),
            sample3DeliverPos = new Pose2d(0.4657, 22.2228, -0.6653),
            submersibleCyclePos = new Pose2d(50.5475, -7.3017, -Math.PI/2),
            cycleDeliverPos = new Pose2d(1.8764, 21.3275, -0.4574);

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (Robot1) RobotFactory.createRobot(hardwareMap);
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
