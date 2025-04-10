package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto2;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

public abstract class SpecimenAuto2Path extends SpecimenAuto2 {

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
        return null;
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return null;
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark1ReadyPos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark1DeliverPos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark2IntakePos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark2DeliverPos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark3IntakePos() {
        return null;
    }

    @Override
    protected Pose2d getSpikeMark3DeliverPos() {
        return null;
    }
}
