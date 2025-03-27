package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;

@TeleOp(name = "PID Tester", group = "Test")
@Config

public class PIDTest extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;
    private Robot1 robot;
    public static double targetPos = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = (Robot1) RobotFactory.createRobot(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {

            robot.setVerticalSlidesHeight(targetPos);

            robot.updateEverything();
            multipleTelemetry.addData("current position", robot.getVerticalSlidesHeightInch());
            multipleTelemetry.addData("PID target position", targetPos);
            multipleTelemetry.update();

        }

    }
}
