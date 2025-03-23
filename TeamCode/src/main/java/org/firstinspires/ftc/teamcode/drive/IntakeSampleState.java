package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class IntakeSampleState {

    public static double MIN_DIST_WITH_SAMPLE = 0.7;
    private RobotBase robot;
    private SampleColor sampleColor = SampleColor.COLOR_UNSPECIFIED;
    private int numOfSamples = 0;

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

        if (numOfSamples > 0) {
            switch (sampleColor) {
                case COLOR_YELLOW:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case COLOR_BLUE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                case COLOR_RED:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    break;
                case COLOR_UNSPECIFIED:
                default:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;
                    break;
            }
        }
    }
    public synchronized String getIntakeColors() {
        return sampleColor.name();
    }

    private boolean colorIsNotSet(SampleColor color) {
        return color == SampleColor.COLOR_UNSPECIFIED;
    }

    public IntakeSampleState(RobotBase robot) {
        this.robot = robot;
    }

    public synchronized boolean isSampleIn() {
        return numOfSamples > 0;
    }

    public void update() {
        int tempNumOfSamples = 0;
        SampleColor tempSampleColor = SampleColor.COLOR_UNSPECIFIED;

        double dist = robot.intakeColorSensor.getDistance(DistanceUnit.INCH);
        if (dist < MIN_DIST_WITH_SAMPLE) {
            tempNumOfSamples = 1;
            tempSampleColor = toHDColor(robot.intakeColorSensor.getNormalizedColors());
        }
        updateInternal(tempNumOfSamples, tempSampleColor);
    }
    private synchronized void updateInternal(int tempNumOfSamples, SampleColor tempSampleColor) {
        numOfSamples = tempNumOfSamples;
        sampleColor = tempSampleColor;
//        setStripLightColors();
    }

    public synchronized int getNumOfSamples() {
        return numOfSamples;
    }

    public synchronized SampleColor getSampleColor() {
        return sampleColor;
    }

    private SampleColor toHDColor(NormalizedRGBA color) {
        HSV hsv = ImageProcessor.ColorToHsv(color);
        if (ImageProcessor.isYellow(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_YELLOW;
        }
        if (ImageProcessor.isRed(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_RED;
        }
        if (ImageProcessor.isBlue(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_BLUE;
        }
        return IntakeSampleState.SampleColor.COLOR_UNSPECIFIED;
    }

    public enum SampleColor {
        COLOR_UNSPECIFIED,
        COLOR_YELLOW,
        COLOR_BLUE,
        COLOR_RED
    }
}
