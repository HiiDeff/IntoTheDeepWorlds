package org.firstinspires.ftc.teamcode.util.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

@Config
public class TeamElementPipeline extends OpenCvPipeline implements VisionProcessor {

    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    public static double MIN_RATIO = 2.5;

    public static int MIN_B = 100, MAX_B = 120, MAX_R = 10, MIN_R = 170, MIN_S = 130, MIN_V = 20;

    public static int THRESHOLD = 6000;

    public static int HEIGHT_PERC = 43;

    private ObjectPos curPos = null;
    private int countL = 0, countR = 0, lean = 0;

    public enum ObjectPos {
        LEFT, NONE, RIGHT
    }

    public enum View {
        LEFT, RIGHT
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) return null;
//        saveImage("/frame_before.png", frame);
        Mat curMat = new Mat();
        Imgproc.cvtColor(frame, curMat, Imgproc.COLOR_RGB2HSV);
        int curCountL = 0, curCountR = 0;
        int[] hRange = {HEIGHT_PERC * curMat.height() / 100, 2 * curMat.height() / 3};
        int mid = (int) (curMat.width() * (0.5 + lean * 1.0 / 6));
        int diff = 2;
        diff = 1;
        for (int i = hRange[0]; i < hRange[1]; i += diff) {
            for (int j = 0; j < curMat.width(); j += diff) {
                double[] hsv = curMat.get(i, j);
                double h = hsv[0], s = hsv[1], v = hsv[2];
                boolean inRange =
                        ((MIN_B < h && h < MAX_B) || h < MAX_R || h > MIN_R) && s > MIN_S && v > MIN_V;
                if (inRange) {
                    if (j < mid) ++curCountL;
                    else ++curCountR;
                    curMat.put(i, j, 255, 255, 255);
                } else {
                    curMat.put(i, j, 0, 0, 0);
                }
            }
        }
//        FtcDashboard.getInstance().sendImage(toBitmap(curMat));
//        saveImage("/frame_after.png", curMat);
        if (curCountR > THRESHOLD) curPos = ObjectPos.RIGHT;
        else if (curCountL > THRESHOLD) curPos = ObjectPos.LEFT;
        else curPos = ObjectPos.NONE;
        countL = curCountL;
        countR = curCountR;
        return curPos;
    }

    private Bitmap toBitmap(Mat mat) {
        Bitmap bmp = null;
        try {
            Mat nm = new Mat();
            Imgproc.cvtColor(mat, nm, Imgproc.COLOR_HSV2RGB);
            Mat tmp = new Mat();
            Imgproc.cvtColor(nm, tmp, Imgproc.COLOR_RGB2RGBA, 4);
            bmp = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(tmp, bmp);
            return bmp;
        }
        catch (CvException e){Log.d("Exception",e.getMessage());}
        return null;
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

    public int getCountL() {
        return countL;
    }

    public int getCountR() {
        return countR;
    }

    public ObjectPos getCurPos() {
        return curPos;
    }

    public void setLean(View view) {
        if (view == View.LEFT) lean = -1;
        else lean = 1;
    }
}
