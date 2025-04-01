package org.firstinspires.ftc.teamcode.auto.defaultauto;

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
            preloadDeliverPos = new Pose2d(2.7589, 21.7425, -0.3637),
            sample1IntakePos = new Pose2d(6.3732, 17.8836, -0.2982),
            sample1DeliverPos = new Pose2d(4.2454, 22.7485, -0.18),
            sample2IntakePos = new Pose2d(6.7833, 21.8209, -0.1044),
            sample2DeliverPos = new Pose2d(0.1476, 22.103, -0.7399),
            sample3IntakePos = new Pose2d(16.9688, 11.0805, 0.6222),
            sample3DeliverPos = new Pose2d(1.4657, 22.2228, -0.6653),
            submersibleCyclePos = new Pose2d(54.5475, -1.3017, -Math.PI/2),
            cycleDeliverPos = new Pose2d(1.8764, 21.3275, -0.4574);

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
