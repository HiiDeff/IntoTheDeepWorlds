package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public abstract class LimelightProcessorBase {

    protected final Limelight3A limelight;
    protected final LimelightConfig LLConfig;
    protected LLResult result = null;
    protected double[] pythonResult = {0};
    protected List<Coordinates> sortedCoordinates;
    public static double MAX_DETECTABLE_DIST_INCH = 30.0;

    public LimelightProcessorBase(Limelight3A limelight, LimelightConfig LLConfig) {
        this.limelight = limelight;
        this.LLConfig = LLConfig;
    }

    public void setMaxDetectableDistInch(double dist) {
        MAX_DETECTABLE_DIST_INCH = dist;
    }

    public synchronized void updateLimelight() {
        result = limelight.getLatestResult();
        if(result!=null) pythonResult = result.getPythonOutput();
        else pythonResult = new double[] {0};
        sortCoordinates();
    }

    public synchronized void sortCoordinates() {
        List<Coordinates> normalizedCoordinates = pythonResultToNormalized(pythonResult);
        List<Coordinates> realWorldCoordinates = normalizedToRealWorld(normalizedCoordinates);
        sortedCoordinates = sortByClosestX(realWorldCoordinates);
    }

    public synchronized Coordinates getObjectPosition() {
        if(sortedCoordinates!=null && !sortedCoordinates.isEmpty()) {
            return sortedCoordinates.get(0);
        }
        else {
            return new Coordinates(0, 0, Math.PI/6);
        }
    }
    public synchronized SampleOrientation getObjectOrientation() {
        if(sortedCoordinates!=null && !sortedCoordinates.isEmpty()) {
            Log.e("edbug sample angle", ""+sortedCoordinates.get(0).getAngle());
            return getOrientationFromAngle(sortedCoordinates.get(0).getAngle());
        }
        else {
            return SampleOrientation.VERTICAL;
        }
    }

    public synchronized LLResult getLLResult() {
        return result;
    }

    // Use normalized because of different resolutions
    protected Coordinates pixelCoordinatesToNormalizedCoordinates(Coordinates pixelCoordinates) {
        double nx = 1.0/(LLConfig.resolutionX/2.0) * (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = 1.0/(LLConfig.resolutionY/2.0) * -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny, pixelCoordinates.getAngle());
    }

    // Prefer normalized over this
    protected Coordinates pixelCoordinatesToCenteredCoordinates(Coordinates pixelCoordinates) {
        double nx = (pixelCoordinates.x - (LLConfig.resolutionX/2.0-0.5));
        double ny = -(pixelCoordinates.y - (LLConfig.resolutionY/2.0-0.5));
        return new Coordinates(nx, ny);
    }

    //receives normalized pixel coordinates as input
    //returns x, y, z, and angle of object in inches and radians. Right (x), forwards (y), and up (z) are positive.
    protected abstract Coordinates normalizedPixelToRealWorldCoordinates(Coordinates pixel);
    protected abstract SampleOrientation getOrientationFromAngle(double angle);

    protected double toRadians(double angDeg) {
        return Math.toRadians(angDeg);
    }
    private List<Coordinates> sortByClosestX(List<Coordinates> realWorldCoordinates) {
        Collections.sort(realWorldCoordinates, new closestComparator());
        return realWorldCoordinates;
    }
    private class closestComparator implements Comparator<Coordinates> {
        @Override
        public int compare(Coordinates o1, Coordinates o2) {
            double comp = Math.abs(o1.getX()) - Math.abs(o2.getX());
            if(comp>0) return 1;
            else if(comp<0) return -1;
            else return 0;
        }
    }
    private List<Coordinates> normalizedToRealWorld(List<Coordinates> normalizedCoordinates) {
        List<Coordinates> realWorld = new ArrayList<>();
        for(Coordinates i: normalizedCoordinates) {
            Coordinates temp = normalizedPixelToRealWorldCoordinates(i);
            if(temp.getY() > MAX_DETECTABLE_DIST_INCH) continue;
            temp.angle = i.angle;
            realWorld.add(temp);
        }
        return realWorld;
    }
    private List<Coordinates> pythonResultToNormalized(double[] pythonResult) {
        List<Coordinates> normalized = new ArrayList<>();
        if(pythonResult.length<=0) return normalized;
        for(int i=1; i<pythonResult[0]*3+1; i+=3){
            if(i+2>=pythonResult.length) break;
            Coordinates cur = new Coordinates(pythonResult[i], pythonResult[i+1], pythonResult[i+2]);
            Log.e("cur "+i,""+cur.getX()+" "+cur.getY()+" "+cur.getAngle());
            normalized.add(pixelCoordinatesToNormalizedCoordinates(cur));
        }
        return normalized;
    }
}
