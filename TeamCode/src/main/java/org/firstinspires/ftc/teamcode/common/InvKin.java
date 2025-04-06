package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import org.firstinspires.ftc.teamcode.util.limelight.Coordinates;

public class InvKin {

    public static double TURRET_LEN = 4.5;
    //this number is the projected length of the turret on the ground when intaking

    public static Coordinates inverseKinematics(Coordinates pos) {
        if(Math.abs(pos.getX())>TURRET_LEN) return new Coordinates(0, 0, 0);

        double y = pos.getY()-Math.sqrt(TURRET_LEN*TURRET_LEN - pos.getX()*pos.getX());
        double angle = Math.toDegrees(Math.asin(pos.getX()/TURRET_LEN));

        Log.e("edbug yPos", ""+y);
        Log.e("edbug xPos", ""+pos.getX());
        Log.e("edbug angle", ""+angle);
        return new Coordinates(pos.getX(),y,angle);
    }
}
