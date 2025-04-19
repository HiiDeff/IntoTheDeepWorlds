package org.firstinspires.ftc.teamcode.auto.sampleauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SAMPLE Auto BLUE FAR FROM SCREEN")
public class SampleAutoBlueFar extends SampleAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }

    @Override
    protected boolean isCloseToScreen() {
        return false;
    }
}
