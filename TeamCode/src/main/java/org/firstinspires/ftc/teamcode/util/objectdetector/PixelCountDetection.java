package org.firstinspires.ftc.teamcode.util.objectdetector;

public class PixelCountDetection extends DetectorBase {

    private static final int THRESHOLD = 200;
    private final int left, width;
    private boolean debugMode = false;
    private Randomization placement = Randomization.NONE;

    public PixelCountDetection(CameraFrameSource source, int left, int right) {
        super(source);
        this.left = left;
        this.width = right - left + 1;
    }

    public void setDebugMode(boolean debugMode) {
        this.debugMode = debugMode;
    }

    public boolean getDebugMode() {
        return debugMode;
    }

    public Randomization getObjectPosition(Cancellable cancellable) {
        int top = 120;
        int height = 240;
        ClipBound bound = new ClipBound(left, top, width, height);
        boolean[][] objectArea = extractObjects(getFrameBitmap(), bound,
                CameraOrientation.LANDSCAPE, ImageProcessor::isBlueOrRed,
                cancellable, debugMode);

        if (objectArea != null) {
            int leftCount = 0;
            int rightCount = 0;
            int left = width / 4;
            int right = 3 * width / 4;
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    if (cancellable.isCancelled()) return placement;
                    if (objectArea[i][j]) {
                        if (j <= left) {
                            leftCount++;
                        } else if (j >= right) {
                            rightCount++;
                        }
                    }
                }
            }

            if (leftCount > THRESHOLD && rightCount > THRESHOLD) {
                placement = Randomization.CENTER;
            } else if (leftCount > THRESHOLD) {
                placement = Randomization.RIGHT;
            } else if (rightCount > THRESHOLD) {
                placement = Randomization.LEFT;
            } else {
                int rand = (int) (Math.random() * 2);
                if (rand == 0) return Randomization.LEFT;
                placement = Randomization.RIGHT;
            }
        }
        return placement;
    }
}
