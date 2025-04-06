package org.firstinspires.ftc.teamcode.util.limelight;

public enum SampleOrientation {
    HORIZONTAL,
    TILTED_LEFT,
    TILTED_RIGHT,
    VERTICAL,
    VERTICAL_INVERSE; //for transfer

    public double toAngleDeg() {
        switch(this) {
            case HORIZONTAL:
                return 0;
            case TILTED_RIGHT:
                return 45;
            case TILTED_LEFT:
                return 135;
            case VERTICAL:
            default:
                return 90;
        }
    }
}
