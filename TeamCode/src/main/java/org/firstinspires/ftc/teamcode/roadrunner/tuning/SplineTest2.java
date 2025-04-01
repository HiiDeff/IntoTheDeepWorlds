package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.Task;

@TeleOp(name = "Spline Test 2")
@Disabled
@Config
public final class SplineTest2 extends LinearOpMode {

    public static double startingTangentDegree = 0;
    public static double firstPosX = 3.2327, firstPosY = 23.6875;
    public static double endingTangentDegree = 90, endX = 56, endY = 0, endHeadingDegrees = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        RobotBase robot = (RobotBase) RobotFactory.createRobot(hardwareMap);

        waitForStart();

        Task test = new RuntimeDrivingTask(
                robot,
                builder -> {
                    Pose2d pos1 = new Pose2d(8.2327, 23.6875, -Math.PI/4);
                    Pose2d pos2 = new Pose2d(endX, endY, Math.toRadians(endHeadingDegrees));
                    return builder
                            .strafeToLinearHeading(pos1.position, pos1.heading)
                            .setTangent(Math.toRadians(startingTangentDegree))
                            .splineToLinearHeading(pos2, Math.toRadians(endingTangentDegree))
                            .build();
                }
        );

        while(opModeIsActive() && !test.perform()) {

        }
        test.cancel();
    }
}
