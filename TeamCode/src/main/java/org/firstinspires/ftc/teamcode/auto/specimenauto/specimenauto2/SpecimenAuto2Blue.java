package org.firstinspires.ftc.teamcode.auto.specimenauto.specimenauto2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "SPECIMEN Auto 2 (push) BLUE")
public class SpecimenAuto2Blue extends SpecimenAuto2Path {
    @Override
    protected boolean isRed() { return false; }
}
