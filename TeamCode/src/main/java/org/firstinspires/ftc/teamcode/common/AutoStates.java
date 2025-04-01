package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;

public class AutoStates {
    private int spikeMarkNumber;
    private int cycleNumber;
    private boolean isRed;
    public int getSpikeMarkNumber() { return spikeMarkNumber; }
    public void setSpikeMarkNumber(int spikeMarkNumber) { this.spikeMarkNumber = spikeMarkNumber; }
    public int getCycleNumber() {
        return cycleNumber;
    }

    public void setCycleNumber(int cycleNumber) {
        this.cycleNumber = cycleNumber;
    }

    public boolean isRed() {
        return isRed;
    }

    public void setAllianceColor(boolean isRed) {
        this.isRed = isRed;
    }
}
