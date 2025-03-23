package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class XGamepad {

    public static double MIN_SAFE_TRIGGER = 0.1;
    public static double MIN_SAFE_JOYSTICK = 0.3;

    private double minSafeTrigger = MIN_SAFE_TRIGGER;
    private double minSafeJoystick = MIN_SAFE_JOYSTICK;

    private GamePad gamepad;

    private int safeLeftTrigger, safeRightTrigger;

    public XGamepad(GamePad gamepad) {
        this.gamepad = gamepad;
    }

    public void setMinSafeTrigger(double minSafeTrigger) {
        this.minSafeTrigger = minSafeTrigger;
    }

    public double getMinSafeTrigger() {
        return minSafeTrigger;
    }

    public void setMinSafeJoystick(double minSafeJoystick) {
        this.minSafeJoystick = minSafeJoystick;
    }

    public double getMinSafeJoystick() {
        return minSafeJoystick;
    }

    public void update() {
        gamepad.update();
        if (gamepad.leftTrigger() > minSafeTrigger) safeLeftTrigger++;
        else safeLeftTrigger = 0;
        if (gamepad.rightTrigger() > minSafeTrigger) safeRightTrigger++;
        else safeRightTrigger = 0;
    }

    public boolean back() {
        return gamepad.back();
    }

    public boolean dpadUp() {
        return gamepad.dpadUp();
    }

    public boolean dpadDown() {
        return gamepad.dpadDown();
    }

    public boolean dpadLeft() {
        return gamepad.dpadLeft();
    }

    public boolean dpadRight() {
        return gamepad.dpadRight();
    }

    public double leftStickX() {
        return gamepad.leftStickX();
    }

    public double leftStickY() {
        return gamepad.leftStickY();
    }

    public double intuitiveLeftStickY() {
        return -gamepad.leftStickY();
    }

    public boolean leftStickButton() {
        return gamepad.leftStickButton();
    }

    public double rightStickX() {
        return gamepad.rightStickX();
    }

    public double rightStickY() {
        return gamepad.rightStickY();
    }

    public double intuitiveRightStickY() {
        return -gamepad.rightStickY();
    }

    public boolean rightStickButton() {
        return gamepad.rightStickButton();
    }

    public boolean sOnceRightStickButton() {
        return !gamepad.back() && !gamepad.start() && gamepad.onceRightStickButton();
    }

    public boolean a() {
        return gamepad.a();
    }

    public boolean sOnceA() {
        return !gamepad.back() && !gamepad.start() && gamepad.onceA();
    }

    public boolean b() {
        return gamepad.b();
    }

    public boolean sOnceB() {
        return !gamepad.back() && !gamepad.start() && gamepad.onceB();
    }

    public boolean x() {
        return gamepad.x();
    }

    public boolean sOnceX() {
        return !gamepad.back() && !gamepad.start() && gamepad.onceX();
    }

    public boolean aOnceY() {
        return gamepad.onceY();
    }

    public boolean sOnceY() {
        return !gamepad.back() && !gamepad.start() && gamepad.onceY();
    }

    public boolean leftBumper() {
        return gamepad.leftBumper();
    }

    public double leftTrigger() {
        return gamepad.leftTrigger();
    }

    public boolean safeLeftTrigger() {
        return safeLeftTrigger > 0;
    }

    public boolean sOnceSafeLeftTrigger() {
        return !gamepad.back() && !gamepad.start() && safeLeftTrigger == 1;
    }

    public boolean rightBumper() {
        return gamepad.rightBumper();
    }

    public double rightTrigger() {
        return gamepad.rightTrigger();
    }

    public boolean safeRightTrigger() {
        return safeRightTrigger > 0;
    }

    public boolean onceComboBumpers() {
        return (gamepad.leftBumper() && gamepad.onceRightBumper()) ||
                (gamepad.onceLeftBumper() && gamepad.rightBumper());
    }
}
