package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Motor Tester", group = "Test")
//@Disabled
@Config

public class MotorTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamePad gp1;

    //DcMotorEx testMotor;
    public DcMotorEx leftFront, rightFront,rightBack, leftBack, vSlideL, vSlideR;
    public Servo leftPTO, rightPTO;

    //public static motorMode mode = motorMode.RUN_WITHOUT_ENCODER;

    public static double RUNTO_POSITION = 0.0;
    public static double POWER = 1;
    public static double LEFT_PTO_OPEN = 0.1;
    public static double RIGHT_PTO_OPEN = 0.92;
    public static double LEFT_PTO_ENGAGE = 0.17;
    public static double RIGHT_PTO_ENGAGE = 0.86;

    public static int vSlidesLEnabled = 1;
    public static int vSlidesREnabled = 1;

    private static boolean vSlidesDown = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new LM1Robot(hardwareMap);

        gp1 = new GamePad(gamepad1);

        leftPTO = hardwareMap.get(Servo.class, "leftPTO");
        rightPTO = hardwareMap.get(Servo.class, "rightPTO");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        vSlideL = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        vSlideL.setDirection(DcMotorEx.Direction.REVERSE);
        vSlideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlideL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        vSlideR = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        vSlideR.setDirection(DcMotorEx.Direction.FORWARD);
        vSlideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlideR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        //EngagePTO(false);

        while (opModeIsActive()) {

            if (gp1.onceA()) {
                EngagePTO(true);
                setPower(POWER);
                telemetry.addData("a pressed", 0);
                telemetry.update();
            } else if (gp1.onceB()) {
                EngagePTO(false);
                setPower(0);
                telemetry.addData("b pressed", 0);
                telemetry.update();

            }
            else if (gp1.onceX())
            {
                if (vSlidesDown) {
                    MoveVSlides(-0.7);
                    vSlidesDown = false;
                }
                else
                {
                    MoveVSlides(0);
                    vSlidesDown = true;
                }
            }
//            telemetry.addData("h pos", vSlideL.getCurrentPosition());
////            telemetry.addData("v pos", verticalSlides.getCurrentPosition());
//            telemetry.update();
            gp1.update();
        }

//        while(opModeIsActive()) {
//            switch (mode) {
//                case RUN_WITHOUT_ENCODER:
//                    setPower();
//                case RUN_TO_POSITION:
//                    runToPos();
//                default:
//                    setPower();
//            }
//        }

    }

    private void EngagePTO(boolean engage)
    {
        if (engage)
        {
            leftPTO.setPosition(LEFT_PTO_ENGAGE);
            rightPTO.setPosition(RIGHT_PTO_ENGAGE);
        }
        else
        {
            leftPTO.setPosition(LEFT_PTO_OPEN);
            rightPTO.setPosition(RIGHT_PTO_OPEN);
        }
    }

    private void MoveVSlides(double power)
    {
        vSlideL.setPower(power);
        vSlideR.setPower(power);
    }

    private void setPower(double power) {
        if (vSlidesLEnabled != 0) {
            vSlideL.setPower(power);

        }
        if (vSlidesREnabled != 0) {
            vSlideR.setPower(power);
        }
        leftFront.setPower(power);
        rightFront.setPower(power);
    }
//    private void setPower() {
//        if(testMotor.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
//            testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        testMotor.setPower(Utils.clamp(POWER, -1.0, 1.0));
//    }
//    private void runToPos() {
//        testMotor.setTargetPosition((int) RUNTO_POSITION);
//        if(testMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
//            testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        }
//        testMotor.setPower(Utils.clamp(POWER, -1.0, 1.0));
//    }
    private enum motorMode {
        RUN_WITHOUT_ENCODER,
        RUN_TO_POSITION;
    }
}
