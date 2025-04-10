package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SPECIMEN Auto RED")
public class SpecimenAutoRed extends SpecimenAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
