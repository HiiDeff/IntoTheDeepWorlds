package org.firstinspires.ftc.teamcode.util.objectdetector;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.util.ArrayList;

public class AprilTagDetector extends AprilTagProcessor {

    public static double yOffset;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>(), detectionsUpdate = new ArrayList<>();
    //private final AprilTagLibrary tagLibrary;

    private long nativeApriltagPtr;
    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();
    private final Object detectionsUpdateSync = new Object();

    private final AprilTagLibrary tagLibrary;

    private final DistanceUnit outputUnitsLength;
    private final AngleUnit outputUnitsAngle;
    private double cx, cy, fx, fy;

    public AprilTagDetector(DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, TagFamily tagFamily, int threads){

        this.tagLibrary = tagLibrary;
        this.outputUnitsLength = outputUnitsLength;
        this.outputUnitsAngle = outputUnitsAngle;
        //nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(tagFamily.ATLibTF.string, 3, threads);
    }


    public double getOffset() { return yOffset; }



    @Override
    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    @Override
    public void setPoseSolver(PoseSolver poseSolver) {

    }

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 0;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
        return null;
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if(needToSetDecimation) {
                setDecimation(decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = detectAprilTags(captureTimeNanos);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // TODO do we need to deep copy this so the user can't mess with it before use in onDrawFrame()?
        return detections;
    }

    ArrayList<AprilTagDetection> detectAprilTags(long captureTimeNanos) {
        long ptrDetectionArray = AprilTagDetectorJNI.runApriltagDetector(nativeApriltagPtr, grey.dataAddr(), grey.width(), grey.height());
        if (ptrDetectionArray != 0) {
            long[] detectionPointers = ApriltagDetectionJNI.getDetectionPointers(ptrDetectionArray);
            ArrayList<AprilTagDetection> detections = new ArrayList<>(detectionPointers.length);

            for (long ptrDetection : detectionPointers) {
                AprilTagMetadata metadata = tagLibrary.lookupTag(ApriltagDetectionJNI.getId(ptrDetection));

                double[][] corners = ApriltagDetectionJNI.getCorners(ptrDetection);

                Point[] cornerPts = new Point[4];
                for (int p = 0; p < 4; p++) {
                    cornerPts[p] = new Point(corners[p][0], corners[p][1]);
                }

                AprilTagPoseRaw rawPose;
                AprilTagPoseFtc ftcPose;

                if (metadata != null) {
                    double[] pose = ApriltagDetectionJNI.getPoseEstimate(
                            ptrDetection,
                            outputUnitsLength.fromUnit(metadata.distanceUnit, metadata.tagsize),
                            fx, fy, cx, cy);

                    // Build rotation matrix
                    float[] rotMtxVals = new float[3 * 3];
                    for (int i = 0; i < 9; i++)
                    {
                        rotMtxVals[i] = (float) pose[3 + i];
                    }

                    rawPose = new AprilTagPoseRaw(
                            pose[0], pose[1], pose[2], // x y z
                            new GeneralMatrixF(3, 3, rotMtxVals)); // R


                }
                else {
                    rawPose = null;
                }

                if (rawPose != null)
                {
                    Orientation rot = Orientation.getOrientation(rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);

                    ftcPose = new AprilTagPoseFtc(
                            rawPose.x,  // x   NB: These are *intentionally* not matched directly;
                            rawPose.z,  // y       this is the mapping between the AprilTag coordinate
                            -rawPose.y, // z       system and the FTC coordinate system
                            -rot.firstAngle, // yaw
                            rot.secondAngle, // pitch
                            rot.thirdAngle,  // roll
                            Math.hypot(rawPose.x, rawPose.z), // range
                            outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.x, rawPose.z)), // bearing
                            outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-rawPose.y, rawPose.z))); // elevation
                }
                else {
                    ftcPose = null;
                }

                double[] center = ApriltagDetectionJNI.getCenterpoint(ptrDetection);

                //TESTING
//                detections.add(new AprilTagDetection(
//                        ApriltagDetectionJNI.getId(ptrDetection),
//                        ApriltagDetectionJNI.getHamming(ptrDetection),
//                        ApriltagDetectionJNI.getDecisionMargin(ptrDetection),
//                        new Point(center[0], center[1]), cornerPts, metadata, ftcPose, rawPose, new Pose3D(0, 0), captureTimeNanos));
            }

            ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray);
            return detections;
        }

        return new ArrayList<>();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    static class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

    static Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsize, int solveMethod)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsize/2, tagsize/2, 0);
        arrayPoints3d[1] = new Point3(tagsize/2, tagsize/2, 0);
        arrayPoints3d[2] = new Point3(tagsize/2, -tagsize/2, 0);
        arrayPoints3d[3] = new Point3(-tagsize/2, -tagsize/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false, solveMethod);

        return pose;
    }

    private aprilTagPose calcPoseVals(AprilTagDetection detection){

        Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double x = detection.rawPose.x;
        double y = detection.rawPose.y;
        double z = detection.rawPose.z;

        double ax = rot.firstAngle;
        double ay = rot.secondAngle;
        double az = rot.thirdAngle;

        double range = Math.sqrt(x*x+y*y);
        double bearing = Math.atan2(-x, y);
        double elevation = Math.atan2(z, y);

        aprilTagPose pose = new aprilTagPose(range, bearing, elevation);
        return pose;
    }

    public class aprilTagPose {
        double RANGE, BEARING, ELEVATION;
        public aprilTagPose(double range, double bearing, double elevation){
            this.RANGE = range;
            this.BEARING = bearing;
            this.ELEVATION = elevation;
        }
    }
}


