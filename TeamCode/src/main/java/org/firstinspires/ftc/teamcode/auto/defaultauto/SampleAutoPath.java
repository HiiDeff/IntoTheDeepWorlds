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
            preloadDeliverPos = new Pose2d(7.7412, 21.2012, -0.381),
            sample1IntakePos = new Pose2d(7.7412, 21.2012, -0.381),
            sample1DeliverPos = new Pose2d(5.8525, 23.1857, -0.1249),
            sample2IntakePos = new Pose2d(5.8525, 23.1857, -0.1249),
            sample2DeliverPos = new Pose2d(5.8525, 23.1857, -0.1249),
            sample3IntakePos = new Pose2d(8.7623,20.5435,0.1907),
            sample3DeliverPos = new Pose2d(6.373, 20.7887, -0.4325),
            submersibleCyclePos = new Pose2d(58.5983, -15.0819, -Math.PI/2),
            cycleDeliverPos = new Pose2d(6.373, 20.7887, -0.4325);

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
