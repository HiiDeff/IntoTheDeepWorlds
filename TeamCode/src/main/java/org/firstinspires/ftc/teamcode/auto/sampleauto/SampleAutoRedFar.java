package org.firstinspires.ftc.teamcode.auto.sampleauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SAMPLE Auto RED FAR FROM SCREEN")
public class SampleAutoRedFar extends SampleAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }

    @Override
    protected boolean isCloseToScreen() {
        return false;
    }
}
