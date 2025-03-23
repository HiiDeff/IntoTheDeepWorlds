package org.firstinspires.ftc.teamcode.util.pid;

public class PIDCoefficients {

    public double minPower;
    public double maxPower;
    public double kP;
    public double kI;
    public double kD;
    public double feedForward;

    public PIDCoefficients(double minPower, double maxPower, double kP, double kI, double kD, double feedForward) {
        this(minPower, maxPower, kP, kI, kD);
        this.feedForward = feedForward;
    }

    public PIDCoefficients(double minPower, double maxPower, double kP, double kI, double kD) {
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
