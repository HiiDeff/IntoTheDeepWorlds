package org.firstinspires.ftc.teamcode.drive_rr;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public class TestRobotPinpoint extends MecanumDrive {

    public static PinpointLocalizer.Params LOCALIZER_PARAMS =
            new PinpointLocalizer.Params();

    public static MecanumDrive.Params TRUE_PARAMS = new MecanumDrive.Params();

    public TestRobotPinpoint(HardwareMap hardwareMap) {
        super();
        setDriveParams();
        super.init(hardwareMap, LocalizerMode.PINPOINT, new Pose2d(0, 0, 0));
    }

    public static void setDriveParams() {
        TRUE_PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        TRUE_PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        TRUE_PARAMS.inPerTick = 0.0022;
        TRUE_PARAMS.lateralInPerTick = TRUE_PARAMS.inPerTick; // 0.0016824700252845181;
        TRUE_PARAMS.trackWidthTicks = 5868;

        // feedforward parameters (in tick units)
        TRUE_PARAMS.kS = 0.9357083339066259;
        TRUE_PARAMS.kV = 0.0003160657851158692;
        TRUE_PARAMS.kA = 0.00007;

        // path profile parameters (in inches)
        TRUE_PARAMS.maxWheelVel = 50;
        TRUE_PARAMS.minProfileAccel = -30;
        TRUE_PARAMS.maxProfileAccel = 50;

        // turn profile parameters (in radians)
        TRUE_PARAMS.maxAngVel = Math.PI; // shared with path
        TRUE_PARAMS.maxAngAccel = Math.PI;

        // path controller gains
        TRUE_PARAMS.axialGain = 10.0;
        TRUE_PARAMS.lateralGain = 8.0;
        TRUE_PARAMS.headingGain = 8.0; // shared with turn

        TRUE_PARAMS.axialVelGain = 0.0;
        TRUE_PARAMS.lateralVelGain = 0.0;
        TRUE_PARAMS.headingVelGain = 0.0; // shared with turn

        PARAMS = TRUE_PARAMS;

        LOCALIZER_PARAMS.perpXTicks = 6.375;
        LOCALIZER_PARAMS.parYTicks = -0.15;
        PinpointLocalizer.PARAMS = LOCALIZER_PARAMS;
    }
}
