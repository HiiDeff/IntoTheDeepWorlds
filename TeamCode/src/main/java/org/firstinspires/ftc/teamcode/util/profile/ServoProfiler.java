package org.firstinspires.ftc.teamcode.util.profile;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoProfiler {

    private final Servo servo;
    private final ProfileConstraints constraints;
    private final ElapsedTime timer;
    private AsymmetricMotionProfile profile;

    private double startTime, targetPos, curPos, delaySeconds;

    public ServoProfiler(Servo servo, ProfileConstraints constraints) {
        this.servo = servo;
        this.constraints = constraints;
        timer = new ElapsedTime();
        profile = null;
    }

    public void setTargetPosition(double target) {
        setTargetPosition(target, 0);
    }

    public void setTargetPosition(double target, double delay) {
        startTime = timer.seconds();
        targetPos = target;
        curPos = servo.getPosition();
        delaySeconds = delay;
        profile = new AsymmetricMotionProfile(curPos, targetPos, constraints);
    }

    public void updatePosition() {
        if (profile == null) return;
        curPos = profile.calculate(Math.max(timer.seconds() - startTime - delaySeconds, 0)).x;
        servo.setPosition(curPos);
    }

    public double getTargetPos() {
        return targetPos;
    }

    public double getCurPos() {
        return curPos;
    }
}
