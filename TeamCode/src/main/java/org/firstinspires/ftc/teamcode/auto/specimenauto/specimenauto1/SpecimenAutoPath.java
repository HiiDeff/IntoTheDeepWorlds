package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class SpecimenAutoPath extends SpecimenAuto {

    public static double CYCLE_INTAKE_X_OFFSET = 0.0,
            CYCLE_INTAKE_Y_OFFSET = 0.0,
            CYCLE_DELIVER_X_OFFSET = 0.0,
            CYCLE_DELIVER_Y_OFFSET = -2,
            CYCLE_INITIAL_Y = 9.0;

    @Override
    protected Pose2d getPreloadDeliverPos() {
        return new Pose2d(29.750,0,0);
    }

    @Override
    protected Pose2d getSpikeMark1IntakePos() {
        return new Pose2d(13.440,-22.517,-0.6598);
    }

    @Override
    protected Pose2d getSpikeMark1DeliverPos() {
        return new Pose2d(11.440,-24.517,-2.30);
    }

    @Override
    protected Pose2d getSpikeMark2IntakePos() {
        return new Pose2d(14.6415,-27.7,-0.8285);
    }

    @Override
    protected Pose2d getSpikeMark2DeliverPos() {
        return new Pose2d(14.6415, -27.7, -2.3335);
    }

    @Override
    protected Pose2d getSpikeMark3IntakePos() {
        return new Pose2d(13.332, -39.037,-0.7888);
    }

    @Override
    protected Pose2d getSpikeMark3DeliverPos() {
        return new Pose2d(13.332,-39.037,-2.3524);
    }

    @Override
    protected Pose2d getCycleIntakePos() {
        return new Pose2d(-0.3,-29.1184,0);
    }

    @Override
    protected Pose2d getCycleDeliverPos() {
        return new Pose2d(29.9051,CYCLE_INITIAL_Y+autoStates.getCycleNumber()*CYCLE_DELIVER_Y_OFFSET,0);
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }
}
