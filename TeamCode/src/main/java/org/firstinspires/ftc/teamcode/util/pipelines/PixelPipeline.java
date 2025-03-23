package org.firstinspires.ftc.teamcode.util.pipelines;

import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.objectdetector.CameraConfig;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

@Config
public class PixelPipeline extends OpenCvPipeline implements VisionProcessor {

    private static final double kCameraHeight = 4.8;
    private static final double kBottomDist = 12.0;
    private static final double kMiddleDist = 31.0;
    private static final double kMiddleWidth = 10.3;
    private static final double xshift = 4, yshift = 0.25;

    public static double ROT_ANGLE = 2.5;
    public static int MIN_LENGTH = 10;

    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    private Pose2d curPos = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) return null;
        Mat rot = Imgproc.getRotationMatrix2D(new Point(frame.width() / 2.0, frame.height() / 2.0),
                ROT_ANGLE, 1);
        Mat dst = new Mat();
        Imgproc.warpAffine(frame, dst, rot, frame.size(), Imgproc.WARP_INVERSE_MAP);
        frame = dst;
//        Log.i("allendebug", "rotated");
//        saveImage("/pixels_before.png", frame);
        Mat curMat = new Mat();
        Imgproc.cvtColor(frame, curMat, Imgproc.COLOR_RGB2HSV);
        int curCountL = 0, curCountR = 0;
        int[] hRange = {0, curMat.height()};
//        int mid = (int) (curMat.width() * (0.5 + v * 1.0 / 6));
        int diff = 2;
        double mxh = 0, mxs = 0, mxv = 0;
        Pose2d fp = null;
        for (int i = hRange[1] - 1; i >= hRange[0]; i -= diff) {
            int len = 0;
            int end = 0;
            for (int j = 0; j < curMat.width(); j += diff) {
                double[] hsv = curMat.get(i, j);
                double h = hsv[0], s = hsv[1], v = hsv[2];
                mxh = Math.max(mxh, h);
                mxs = Math.max(mxs, s);
                mxv = Math.max(mxv, v);
                // yellow, white
                boolean inRange = (15 <= h && h <= 30 && s >= 50) || (s <= 50 && v >= 200);
                if (inRange) {
                    len++;
                    curMat.put(i, j, 255, 255, 255);
                } else {
                    end = j;
                    if (len > MIN_LENGTH && fp == null) {
                        Pose2d objPos = new Pose2d(end - len * 0.5 - frame.width() * 0.5,
                                frame.height() * 0.5 - i - 1, 0.0);
                        Log.i("allendebug",String.format("checking %3.1f, %3.1f",
                                objPos.position.x, objPos.position.y));
                        Pose2d fieldPos = (new CameraConfig(kCameraHeight, kBottomDist,
                                kMiddleDist, kMiddleWidth, frame.width(), frame.height(), xshift,
                                yshift)).imagePosToFieldPos(objPos);
                        Log.i("allendebug", String.format("pos in field: %3.1f, %3.1f",
                                fieldPos.position.x, fieldPos.position.y));
                        fp = fieldPos;
                    }
                    len = 0;
                    curMat.put(i, j, 0, 0, 0);
                }
            }
        }
//        Log.i("allendebugmx", mxh + " " + mxs + " " + mxv);
//        saveImage("/pixels_after.png", curMat);
        if (fp == null) fp = new Pose2d(0,0,0);
        curPos = fp;
        return fp;
    }

    private void saveImage(String fileName, Mat mat) {
        File file = new File(captureDirectory, fileName);
        saveMatToDiskFullPath(mat, captureDirectory.getAbsolutePath() + fileName);
        Log.i("allendebug", "saved in " + captureDirectory.getAbsolutePath() + fileName);
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
}
