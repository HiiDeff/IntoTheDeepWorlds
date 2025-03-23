package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Stack;

@Config
public class LimelightSampleDetector extends LimelightProcessorBase {

    //in degrees
    public static double HORIZONTAL_THRESHOLD_MIN = 10.0, HORIZONTAL_THRESHOLD_MAX = 170.0;
    public static double VERTICAL_THRESHOLD_MIN = 70.0, VERTICAL_THRESHOLD_MAX = 110.0;
    public static double SAMPLE_HEIGHT_INCH = 1.4;

    public LimelightSampleDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }

    @Override
    protected Coordinates normalizedPixelToRealWorldCoordinates(Coordinates pixel, double pixelHeight) {
        //Relative to the Limelight:
        // y dist
        double objectAngleWithHorizontal = pixel.y * (LLConfig.verticalFOV/2.0) + LLConfig.angleWithHorizontal; //pixel.y should be negative if object is lower than center of camera fov
        double yDistInch = (LLConfig.zOffset-pixelHeight) * Math.tan(toRadians(90+objectAngleWithHorizontal));
        // x dist
        double yzPlaneProjectedDist = Math.sqrt(yDistInch*yDistInch + (LLConfig.zOffset-pixelHeight)*(LLConfig.zOffset-pixelHeight)); // projected distance on the yz plane
        double xAngleWithObject = pixel.x * (LLConfig.horizontalFOV/2.0);
        double xDistInch = yzPlaneProjectedDist * Math.tan(toRadians(xAngleWithObject));

        // account for Limelight offset to center of robot:
        yDistInch += LLConfig.yOffset;
        xDistInch += LLConfig.xOffset;

//        Log.i("pixel x", ""+pixel.x);
//        Log.i("pixel y", ""+pixel.y);
//        Log.i("objectAngleWithHorizontal",""+objectAngleWithHorizontal);
//        Log.i("xAngleWithObject", ""+xAngleWithObject);

        double angleToObject = Math.atan(xDistInch/yDistInch);
        return new Coordinates(xDistInch, yDistInch, angleToObject);
    }

    @Override
    protected double calcObjectAngleDegrees(List<List<Double>> targetCorners) {

        if(targetCorners.size()<=3) return 90.0;

        double topPixelY = (double)1e5, topPixel2Y = (double)1e5;
        int topPixel = 0, topPixel2 = 0;
        for(int i=0; i<targetCorners.size(); i++) {
            double py = targetCorners.get(i).get(1);
            if(py <= topPixelY){
                topPixel2Y = topPixelY;
                topPixelY = py;
                topPixel2 = topPixel;
                topPixel = i;
            }else if(py <= topPixel2Y) {
                topPixel2Y = py;
                topPixel2 = i;
            }
        }

        List<Coordinates> tC = new ArrayList<>();
        for(int i=0; i<targetCorners.size(); i++) {
            tC.add(
                    normalizedPixelToRealWorldCoordinates(
                            pixelCoordinatesToNormalizedCoordinates(
                                    new Coordinates(targetCorners.get(i).get(0), targetCorners.get(i).get(1))
                            ), ((i==topPixel||i==topPixel2) ? SAMPLE_HEIGHT_INCH:0.0)
                    )
            );
        }
        for(int i=0; i<tC.size(); i++){
            Log.i("corner",tC.get(i).getX()+" "+tC.get(i).getY());
        }

        List<Coordinates> hull = computeConvexHull(tC);

        for(Coordinates i: hull) {
            Log.i("corner hull points", i.getX()+" "+i.getY());
        }

        double maxDist = 0.0;
        Coordinates firstPoint = null, secondPoint = null;
        for (int i = 0; i < hull.size(); i++) {
            Coordinates p1 = hull.get(i);
            Coordinates p2 = hull.get((i + 1) % hull.size()); // Wrap around for last edge

            double dist = Math.sqrt(
                    (p1.getY()-p2.getY())*(p1.getY()-p2.getY())
                    + (p1.getX()-p2.getX())*(p1.getX()-p2.getX())
            );
            if (dist > maxDist) {
                maxDist = dist;
                firstPoint = p1;
                secondPoint = p2;
            }
        }

        if(maxDist==0.0 || firstPoint == null || secondPoint == null) {
            return 90.0;
        }

        Log.i("first corner point",firstPoint.getX()+" "+firstPoint.getY());
        Log.i("second corner point",secondPoint.getX()+" "+secondPoint.getY());

        double angle = Math.atan2(firstPoint.getY()-secondPoint.getY(),firstPoint.getX()-secondPoint.getX());
        if(angle<0) angle += Math.PI;
        angle = Math.toDegrees(angle);

        Log.i("angle", angle+"");

        return angle;
    }

    private static List<Coordinates> computeConvexHull(List<Coordinates> points) {
        if(points.size()<3) return new ArrayList<>();

        Coordinates anchor = Collections.min(points, Comparator.comparingDouble(Coordinates::getY).thenComparingDouble(Coordinates::getX));
        points.sort((p1, p2) -> {
            double angle1 = Math.atan2(p1.getY() - anchor.getY(), p1.getX() - anchor.getX());
            double angle2 = Math.atan2(p2.getY() - anchor.getY(), p2.getX() - anchor.getX());
            return Double.compare(angle1, angle2);
        });

        Stack<Coordinates> hull = new Stack<>();
        for (Coordinates p : points) {
            while (hull.size() > 1 && crossProduct(hull.get(hull.size() - 2), hull.peek(), p) <= 0) {
                hull.pop();
            }
            hull.push(p);
        }
        return new ArrayList<>(hull);
    }

    private static double crossProduct(Coordinates A, Coordinates B, Coordinates C) {
        return (B.getX() - A.getX()) * (C.getY() - B.getY()) - (B.getY() - A.getY()) * (C.getX() - B.getX());
    }

    @Override
    protected ObjectOrientation getOrientationFromAngle(double angle) { //angle from 0-180 degrees
        if(angle <= HORIZONTAL_THRESHOLD_MIN || angle >= HORIZONTAL_THRESHOLD_MAX) {
            return ObjectOrientation.HORIZONTAL;
        }else if(angle <= VERTICAL_THRESHOLD_MIN) {
            return ObjectOrientation.TILTED_RIGHT;
        }else if(angle >= VERTICAL_THRESHOLD_MAX) {
            return ObjectOrientation.TILTED_LEFT;
        }else {
            return ObjectOrientation.VERTICAL;
        }
    }
}
