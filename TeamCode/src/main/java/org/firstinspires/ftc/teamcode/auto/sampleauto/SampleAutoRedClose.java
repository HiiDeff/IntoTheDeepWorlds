package org.firstinspires.ftc.teamcode.auto.sampleauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SAMPLE Auto RED CLOSE TO SCREEN")
public class SampleAutoRedClose extends SampleAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }

    @Override
    protected boolean isCloseToScreen() {
        return true;
    }
}
