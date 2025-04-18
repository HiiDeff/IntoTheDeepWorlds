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
            CYCLE_INITIAL_Y = -3.0;

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadDeliverPos() {
        return new Pose2d(29.466, 11, 0.0);
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return new Pose2d(0.5, -29.582, 0.0);
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return new Pose2d(30, CYCLE_INITIAL_Y + autoStates.getCycleNumber() * CYCLE_DELIVER_Y_OFFSET, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark1ReadyPos() {
        return new Pose2d(16.788, -24.410, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakeLeftPos() {
        return new Pose2d(45, -26, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return new Pose2d(49, -37, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark1DeliverPos() {
        return new Pose2d(10, -26, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark2IntakeLeftPos() {
        return new Pose2d(45, -32, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark2IntakePos() {
        return new Pose2d(49, -50, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark2DeliverPos() {
        return new Pose2d(7, -41, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark3IntakeLeftPos() {
        return new Pose2d(55, -57, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark3IntakePos() {
        return new Pose2d(58, -59, 0.0);
    }

    @Override
    protected Pose2d getSpikeMark3DeliverPos() {
        return new Pose2d(7, -50, 0.0);
    }
}