package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

public class Utils {

    public static final double EPSILON = 0.000001;

    // Meant to be used as global utils
    private Utils() {
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }

    // Estimates the max travel time for rotating a servo with the servo spec and travel distance.
    // This method assumes the load is trivial and servo rotation direction doesn't change the
    // travel time too much.
    public static int getEstimatedServoTime(int rpm, int maxRotationDegree, double travelDist) {
        double secondsPerRotation = 60.0 / rpm;
        double secondsForFullRange = maxRotationDegree / 360.0 * secondsPerRotation;
        RobotLog.ee("lswu-degug", "waiting time: %d, diff: %f",
                (int) (Math.abs(travelDist) * secondsForFullRange * 1000), travelDist);
        return (int) (Math.abs(travelDist) * secondsForFullRange * 1000);
    }

    // Estimates the max travel time with consideration of servo load and servo travel direction.
    // If a negative travelDist means going up, set negativeIsUp to true.
    public static int getEstimatedServoTimeHeavyLoad(int rpm, int maxRotationDegree,
            double travelDist, boolean negativeIsUp) {
        int basicTime = getEstimatedServoTime(rpm, maxRotationDegree, travelDist);
        if (isGoingUp(travelDist, negativeIsUp)) {
            basicTime *= 2;
        }
        return basicTime;
    }

    private static boolean isGoingUp(double travelDist, boolean negativeIsUp) {
        return (travelDist < 0 && negativeIsUp) || (travelDist > 0 && !negativeIsUp);
    }
}
