package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SPECIMEN Auto 1 (intake) BLUE")
public class SpecimenAutoBlue extends SpecimenAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
