package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
@Autonomous(name = "SPECIMEN Auto (RELIABLE 5)")
public class SpecimenAuto4Path extends SpecimenAuto4 {

    public static double CYCLE_INTAKE_X_OFFSET = 0.0,
            CYCLE_INTAKE_Y_OFFSET = 0.0,
            CYCLE_DELIVER_X_OFFSET = 0.0,
            CYCLE_DELIVER_Y_OFFSET = 1.5,
            CYCLE_INITIAL_Y = -0.0;

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadDeliverPos() {
        return new Pose2d(30,-5,0.0);
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return new Pose2d(-0.3,-29.582,0.0);
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return new Pose2d(29.0,CYCLE_INITIAL_Y+autoStates.getCycleNumber()*CYCLE_DELIVER_Y_OFFSET,0.0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return new Pose2d(28, -32, -0.7357);
    }

    @Override
    protected Pose2d getSpikeMark1DeliverPos() {
        return new Pose2d(28.82, -31.82, -2.883);
    }

    @Override
    protected Pose2d getSpikeMark2IntakePos() {
        return new Pose2d(28.5, -41, -0.7014);
    }

    @Override
    protected Pose2d getSpikeMark2DeliverPos() {
        return new Pose2d(27.96, -42.65, -2.9);
    }

    @Override
    protected Pose2d getSpikeMark3IntakePos() {
        return new Pose2d(29, -50.62, -0.7952);
    }

    @Override
    protected Pose2d getSpikeMark3DeliverPos() {
        return new Pose2d(19.22, -39.28, -2.74);
    }
}