package org.firstinspires.ftc.teamcode.util.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public abstract class LimelightProcessorBase {

    protected final Limelight3A limelight;
    protected final LimelightConfig LLConfig;
    protected LLResult result = null;

    public LimelightProcessorBase(Limelight3A limelight, LimelightConfig LLConfig) {
        this.limelight = limelight;
        this.LLConfig = LLConfig;
    }

    public synchronized void updateLimelight() {
        result = limelight.getLatestResult();
    }

    public synchronized Coordinates getObjectPosition(double objectCenterHeight) {
        if(result!=null){
            double[] pythonResult = result.getPythonOutput();
            if(pythonResult != null && pythonResult[0]>0) {
                Coordinates pixelCoordinates = new Coordinates(pythonResult[1], pythonResult[2]);
                Coordinates normalizedCoordinates = pixelCoordinatesToNormalizedCoordinates(pixelCoordinates);
                return normalizedPixelToRealWorldCoordinates(normalizedCoordinates, objectCenterHeight);
            }
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            LLResultTypes.ColorResult firstResult = colorResults.get(0);
//            Coordinates pixelCoordinates = new Coordinates(firstResult.getTargetXPixels(), firstResult.getTargetYPixels());
//            Coordinates normalizedCoordinates = pixelCoordinatesToNormalizedCoordinates(pixelCoordinates);
//            return normalizedPixelToRealWorldCoordinates(normalizedCoordinates, objectCenterHeight);
        }
        return new Coordinates(10, 0);
    }
    public synchronized ObjectOrientation getObjectOrientation() {
        if(result!=null) {
            double[] pythonResult = result.getPythonOutput();
            if(pythonResult != null && pythonResult[0]>0) {
                double angle = pythonResult[3];
                return getOrientationFromAngle(angle);
            }
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            double[] test = result.getPythonOutput();
//            LLResultTypes.ColorResult firstResult = colorResults.get(0);
//            List<List<Double>> corners = firstResult.getTargetCorners();
//            double angle = calcObjectAngleDegrees(corners);
//            return getOrientationFromAngle(angle);
        }
        return ObjectOrientation.VERTICAL;
    }

    public synchronized LLResult getLLResult() {
        return result;
    }

    // Use normalized because of different resolutions
    protected Coordinates pixelCoordinatesToNormalizedCoordinates(Coordinates pixelCoordinates) {
        double nx = 1.0/(LLConfig.resolutionX/2.0) * (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = 1.0/(LLConfig.resolutionY/2.0) * -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny);
    }

    // Prefer normalized over this
    protected Coordinates pixelCoordinatesToCenteredCoordinates(Coordinates pixelCoordinates) {
        double nx = (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny);
    }

    //receives normalized pixel coordinates as input
    //returns x, y, z, and angle of object in inches and radians. Right (x), forwards (y), and up (z) are positive.
    protected abstract Coordinates normalizedPixelToRealWorldCoordinates(Coordinates pixel, double pixelHeight);
    //receives LLResult.getTargetCorners() as input
    protected abstract double calcObjectAngleDegrees(List<List<Double>> targetCorners);
    protected abstract ObjectOrientation getOrientationFromAngle(double angle);

    protected double toRadians(double angDeg) {
        return Math.toRadians(angDeg);
    }
}
