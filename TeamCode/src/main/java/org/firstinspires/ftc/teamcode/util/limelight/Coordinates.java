package org.firstinspires.ftc.teamcode.util.limelight;

public class Coordinates {

    public double x, y, z;
    public double angle;
    public Coordinates(double x, double y){
        this.x = x;
        this.y = y;
        this.z = 0;
        this.angle = 0.0;
    }
    public Coordinates(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.z = 0;
        this.angle = angle;
    }
    public Coordinates(double x, double y, double z, double angle) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.angle = angle;
    }
    public String toString() {
        return "("+x+" "+y+" "+z+" "+angle+")";
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
    public double getAngle() {
        return angle;
    }

}
