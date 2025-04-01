package org.firstinspires.ftc.teamcode.drive_rr;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public class TestRobotPinpoint extends PinpointDrive {

    public static PinpointDrive.Params PINPOINT_PARAMS =
            new PinpointDrive.Params();

    public static MecanumDrive.Params TRUE_PARAMS = new MecanumDrive.Params();

    public TestRobotPinpoint(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        setDriveParams();
        super.init(hardwareMap);
    }

    public static void setDriveParams() {
        TRUE_PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        TRUE_PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        TRUE_PARAMS.inPerTick = 1.0;
        TRUE_PARAMS.lateralInPerTick = 1.213675706866705135603000577034; //(30.705,18.643) (51.738,35.973) // calculated myself (Trust this value). Heavy robot prevents lateral ramp from working well
        TRUE_PARAMS.trackWidthTicks = 14.245478953115196;

        // feedforward parameters (in tick units)
        TRUE_PARAMS.kS = 1.2800645469636986;
        TRUE_PARAMS.kV = 0.13301959837487504;
        TRUE_PARAMS.kA = 0.03;

        // path profile parameters (in inches)
        TRUE_PARAMS.maxWheelVel = 60;
        TRUE_PARAMS.minProfileAccel = -40;
        TRUE_PARAMS.maxProfileAccel = 60;

        // turn profile parameters (in radians)
        TRUE_PARAMS.maxAngVel = Math.PI; // shared with path
        TRUE_PARAMS.maxAngAccel = Math.PI;

        // path controller gains
        TRUE_PARAMS.axialGain = 5.0;
        TRUE_PARAMS.lateralGain = 5.0; //10
        TRUE_PARAMS.headingGain = 3.0; // shared with turn //4

        TRUE_PARAMS.axialVelGain = 0.0;
        TRUE_PARAMS.lateralVelGain = 0.0;
        TRUE_PARAMS.headingVelGain = 0.0; // shared with turn

        MecanumDrive.PARAMS = TRUE_PARAMS;

        PINPOINT_PARAMS.xOffset = 3.1091754774386136; //3.25 measured, but RR more accurate
        PINPOINT_PARAMS.yOffset = 1.9439441107387374; //1.625 measured, but RR more accurate

        PINPOINT_PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        PINPOINT_PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PINPOINT_PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        PinpointDrive.PARAMS = PINPOINT_PARAMS;
    }
}
