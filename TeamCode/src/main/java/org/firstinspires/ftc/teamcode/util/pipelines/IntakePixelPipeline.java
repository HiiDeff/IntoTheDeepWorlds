package org.firstinspires.ftc.teamcode.util.pipelines;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakePixelPipeline extends OpenCvPipeline implements VisionProcessor {

    private static final double K_CAMERA_HEIGHT = 4.8;
    private static final double K_BOTTOM_DIST = 12.0;
    private static final double K_MIDDLE_DIST = 31.0;
    private static final double K_MIDDLE_WIDTH = 10.3;
    private static final double X_SHIFT = 4, Y_SHIFT = 0.25;

    private static final double HEIGHT_RECIPROCAL = 1.0 / K_CAMERA_HEIGHT;
    private static final double BOTTOM_ANGLE = Math.atan(K_BOTTOM_DIST / HEIGHT_RECIPROCAL);
    private static final double MID_ANGLE = Math.atan(K_MIDDLE_DIST / HEIGHT_RECIPROCAL);
    private static final double HALF_IMAGE_ANGLE = MID_ANGLE - BOTTOM_ANGLE;

    private static final double CAMERA_ROT_ANGLE = 2.5;
    private static final int SKIP = 2;
    private static final int MIN_LENGTH = 24 / SKIP;

    private Pose2d curPos = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat mat, long captureTimeNanos) {
        if (mat == null) return null;
        // Rotate image so that it is straight:
        Mat rot = Imgproc.getRotationMatrix2D(
                new Point(mat.width() / 2.0, mat.height() / 2.0), CAMERA_ROT_ANGLE, 1);
        Mat frame = new Mat();
        Imgproc.warpAffine(mat, frame, rot, mat.size(), Imgproc.WARP_INVERSE_MAP);
        // Convert to HSV:
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Pose2d pos = null;
        for (int i = hsvMat.height() - 1; i >= 0; i -= SKIP) {
            if (pos != null) break;
            int len = 0;
            for (int j = 0; j < hsvMat.width(); j += SKIP) {
                if (pos != null) break;
                // <H range: 0-130, S range: 0-255, V range: 0-255>
                double[] hsv = hsvMat.get(i, j);
                boolean inRange = (15 <= hsv[0] && hsv[0] <= 30 && hsv[1] >= 50) // yellow
                        || (hsv[1] <= 50 && hsv[2] >= 200); // white
                if (inRange) len++;
                else if (len < MIN_LENGTH) len = 0;
                else {
                    Pose2d imgPos = new Pose2d(j - len * 0.5 - frame.width() * 0.5,
                            frame.height() * 0.5 - i - 1, 0.0);
                    pos = imageToField(imgPos, frame.width(), frame.height());
                }
            }
        }
        if (pos == null) pos = new Pose2d(0, 0, 0);
        curPos = pos;
        return pos;
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {}

    public Pose2d getCurPos() {
        return curPos;
    }

    public void reset() {
        curPos = null;
    }

    public static Pose2d imageToField(Pose2d pos, int imageWidth, int imageHeight) {
        double halfImageWidth = imageWidth * 0.5;
        double halfImageHeight = imageHeight * 0.5;
        double objAngle = HALF_IMAGE_ANGLE * pos.position.y / halfImageHeight;
        double x = K_CAMERA_HEIGHT * Math.tan(MID_ANGLE + objAngle);
        double y = -K_MIDDLE_WIDTH * x * pos.position.x / K_MIDDLE_DIST / halfImageWidth;
        return new Pose2d(x - X_SHIFT, y - Y_SHIFT, 0.0);
    }
}
