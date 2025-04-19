package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto2;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.IntakeSampleState;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class SpecimenAuto2Base extends LinearOpMode {

    public static int AA_TOTAL_TIME_MILLIS = 30000;

    public static int AA_NUM_OF_SPIKE_MARKS = 3,
            AA_NUM_OF_CYCLES = 5;

    public static int INIT_DELAY_TIME = 0;
    public static int AA_TIME_FOR_A_CYCLE = 0000;
    public static int AA_TIME_FOR_PARK = 0000; //0 for now because prioritize delivering a sample over 3 point hang

    // Moving velocity constraints
    public static double HIGH_SPEED = 35;
    public static double LOW_SPEED = 30;

    protected final ElapsedTime timer = new ElapsedTime();
    protected RobotBase robot;

    protected AutoState state = AutoState.START;
    protected AutoStates autoStates = new AutoStates();
    protected SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap);
        resetRobot();
        robot.setLimelightPipeline(isRed()? IntakeSampleState.SampleColor.COLOR_RED : IntakeSampleState.SampleColor.COLOR_BLUE, false);
        robot.setLimelightMaxDetectionDistInch(15.0);

        sensorUpdateThread = new SensorUpdateThread(robot);

        while (opModeInInit()) {
            telemetry.addData("initializing", 1);
            telemetry.update();
        }

        state = AutoState.START;
        Task task = createStartTask();
        timer.reset();
        sensorUpdateThread.start();
        robot.startLimelight();

        ElapsedTime cycleTimer = new ElapsedTime();

        boolean done = false;
        while (opModeIsActive() && !done && task != null) {
            robot.updateEverything();
            robot.updateLimelight();
            Log.i("edbug specimen cycles", robot.getLimelightObjectPosition().toString());
            if(timeToFinish() && state != AutoState.FINISH) {
                task.cancel();
                task = createFinishTask();
            } else if(task.perform()) {
                if(state == AutoState.START) {
                    task = createCycleWithIntakeTask();
                }else if(hasSpikeMarksLeft()) {
                    autoStates.setSpikeMarkNumber(autoStates.getSpikeMarkNumber() + 1);
                    Log.i("spike mark number", ""+autoStates.getSpikeMarkNumber());
                    task = createSpikeMarkTask();
                } else if(hasTimeForOneMoreCycle()) {
                    autoStates.setCycleNumber(autoStates.getCycleNumber() + 1);
                    task = createCycleTask();
                } else if(state != AutoState.FINISH) {
                    task = createFinishTask();
                } else {
                    done = true;
                }
            }
            telemetry.addData("status", "running");
            telemetry.addData("angle", robot.pose.heading.toDouble());
            telemetry.addData("x", robot.pose.position.x);
            telemetry.addData("y", robot.pose.position.y);
            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.update();

            Log.i("edbug loop times", cycleTimer.milliseconds()+"");
            cycleTimer.reset();
        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private boolean hasSpikeMarksLeft() {
        return autoStates.getSpikeMarkNumber() < AA_NUM_OF_SPIKE_MARKS;
    }

    private boolean hasTimeForOneMoreCycle() {
        // Go ahead if there are still 10 seconds left and
        return autoStates.getCycleNumber() < AA_NUM_OF_CYCLES &&
                timer.milliseconds() < AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_A_CYCLE;
    }

    private void resetRobot() {
        robot.autoInit();
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);

    protected abstract Task createStartTask();
    protected abstract Task createCycleWithIntakeTask();
    protected abstract Task createSpikeMarkTask();
    protected abstract Task createCycleTask();
    protected abstract Task createFinishTask();
    protected abstract boolean isRed();
    protected double toR(double degree) {
        return Math.toRadians(degree);
    }

    // Reverse the direction and convert the degree to radius.
    protected double toOppositeR(double degree) {
        return Math.toRadians(degree + 180);
    }

    protected enum AutoState {
        START,
        CYCLE_WITH_INTAKE,
        SPIKE_MARKS,
        CYCLE,
        FINISH
    }
}
