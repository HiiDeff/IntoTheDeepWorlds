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
        TRUE_PARAMS.lateralInPerTick = 0.6917565598; //(20.248,8.229) (32.5204,16.6771) -> 8.4481/12.2724    // calculated myself (Trust this value). Heavy robot prevents lateral ramp from working well
        // (27.5776-12.9116)/(47.2896-26.0885) = 14.666/21.2011 = 0.6917565598
        //RR gave me 0.5284068899336227
        TRUE_PARAMS.trackWidthTicks = 14.931922905302441;

        // feedforward parameters (in tick units)
        TRUE_PARAMS.kS = 1.163151239887608;
        TRUE_PARAMS.kV = 0.13158833814790233;
        TRUE_PARAMS.kA = 0.04;

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

        PINPOINT_PARAMS.xOffset = -7.1;
        PINPOINT_PARAMS.yOffset = 3.25;

        PINPOINT_PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        PINPOINT_PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PINPOINT_PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        PinpointDrive.PARAMS = PINPOINT_PARAMS;
    }
}
