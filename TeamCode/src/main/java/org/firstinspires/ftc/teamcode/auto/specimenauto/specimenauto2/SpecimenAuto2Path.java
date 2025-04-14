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
        return new Pose2d(29.466,3.593,0.0);
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return new Pose2d(0.5,-29.582,0.0);
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return new Pose2d(28.529,CYCLE_INITIAL_Y+autoStates.getCycleNumber()*CYCLE_DELIVER_Y_OFFSET,0.0);
    }

    @Override
    protected Pose2d getSpikeMark1ReadyPos() {
        return new Pose2d(20.788,-25.410,0.0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return new Pose2d(50.12,-30.46, 0.0);
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
        return new Pose2d(5.556,-46.31,0.0);
    }
}
