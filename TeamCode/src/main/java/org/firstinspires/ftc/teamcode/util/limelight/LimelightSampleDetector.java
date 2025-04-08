package org.firstinspires.ftc.teamcode.util.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Config
public class LimelightSampleDetector extends LimelightProcessorBase {

    //in degrees
    public static double HORIZONTAL_THRESHOLD_MIN = 10.0, HORIZONTAL_THRESHOLD_MAX = 170.0;
    public static double VERTICAL_THRESHOLD_MIN = 70.0, VERTICAL_THRESHOLD_MAX = 110.0;
    public static double TURRET_DIST_FROM_ROBOT_CENTER = 6.0;
    public static double PIXEL_HEIGHT = 1.0;

    public LimelightSampleDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }

    @Override
    protected Coordinates normalizedPixelToRealWorldCoordinates(Coordinates pixel) {
        //Relative to the Limelight:
        // y dist
        double objectAngleWithHorizontal = pixel.y * (LLConfig.verticalFOV/2.0) + LLConfig.angleWithHorizontal; //pixel.y should be negative if object is lower than center of camera fov
        double yDistInch = (LLConfig.zOffset-PIXEL_HEIGHT) * Math.tan(toRadians(90+objectAngleWithHorizontal));

        yDistInch += (yDistInch-12.0) * LLConfig.errorScalerY;

        // x dist
        double yzPlaneProjectedDist = Math.sqrt(yDistInch*yDistInch + (LLConfig.zOffset-PIXEL_HEIGHT)*(LLConfig.zOffset-PIXEL_HEIGHT)); // projected distance on the yz plane
        double xAngleWithObject = pixel.x * (LLConfig.horizontalFOV/2.0);
        double xDistInch = yzPlaneProjectedDist * Math.tan(toRadians(xAngleWithObject));

        xDistInch += (xDistInch) * LLConfig.errorScalerX;


        // account for Limelight offset to center of robot:
        yDistInch += LLConfig.yOffset;
        xDistInch += LLConfig.xOffset;

//        Log.i("pixel x", ""+pixel.x);
//        Log.i("pixel y", ""+pixel.y);
//        Log.i("objectAngleWithHorizontal",""+objectAngleWithHorizontal);
//        Log.i("xAngleWithObject", ""+xAngleWithObject);

        double angleToObject = Math.atan(xDistInch/(TURRET_DIST_FROM_ROBOT_CENTER+yDistInch));


        return new Coordinates(xDistInch, yDistInch, angleToObject);
    }

    @Override
    protected SampleOrientation getOrientationFromAngle(double angle) { //angle from 0-180 degrees
        if(angle <= HORIZONTAL_THRESHOLD_MIN || angle >= HORIZONTAL_THRESHOLD_MAX) {
            return SampleOrientation.HORIZONTAL;
        }else if(angle <= VERTICAL_THRESHOLD_MIN) {
            return SampleOrientation.TILTED_RIGHT;
        }else if(angle >= VERTICAL_THRESHOLD_MAX) {
            return SampleOrientation.TILTED_LEFT;
        }else {
            return SampleOrientation.VERTICAL;
        }
    }
}
