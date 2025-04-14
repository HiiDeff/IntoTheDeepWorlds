package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
@Autonomous(name = "SPECIMEN Auto (reliable 5 clip)")
public class SpecimenAuto3Path extends SpecimenAuto3 {

    public static double CYCLE_INTAKE_X_OFFSET = 0.0,
            CYCLE_INTAKE_Y_OFFSET = 0.0,
            CYCLE_DELIVER_X_OFFSET = 0.0,
            CYCLE_DELIVER_Y_OFFSET = 1.5,
            CYCLE_INITIAL_Y = -1.0;

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadDeliverPos() {
        return new Pose2d(31,-3,0.0);
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return new Pose2d(-0.4,-29.582,0.0);
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return new Pose2d(29.8,CYCLE_INITIAL_Y+autoStates.getCycleNumber()*CYCLE_DELIVER_Y_OFFSET,0.0);
    }

    @Override
    protected Pose2d getSpikeMark1ReadyPos() {
        return new Pose2d(22.788,-26.410,0.0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return new Pose2d(50.12,-28.46, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark1DeliverPos() {
        return new Pose2d(5.83,-30.46, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark2IntakePos() {
        return new Pose2d(60.22,-52.235,0.0);
    }

    @Override
    protected Pose2d getSpikeMark2DeliverPos() {
        return new Pose2d(5.698,-40.235,0.0);
    }

    @Override
    protected Pose2d getSpikeMark3IntakePos() {
        return new Pose2d(56.76,-56.89,0.0);
    }

    @Override
    protected Pose2d getSpikeMark3DeliverPos() {
        return new Pose2d(3.556,-46.31,0.0);
    }
}
