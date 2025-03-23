package org.firstinspires.ftc.teamcode.util.objectdetector;


import com.acmerobotics.roadrunner.Pose2d;

public class CameraConfig {

    // Camera center height above the ground
    private double cameraHeight;
    // Distance between the camera position (projected on the ground) and bottom of image (closer
    // to the camera)
    private double bottomDist;
    // Distance between the camera position (projected on the ground) and center of image
    private double middleDist;
    // Half horizontal size of ground corresponding to center horizontal line of image
    private double middleWidth;
    // The half of the image size (horizontal)
    private double imageHalfWidth;
    // The half of the image size (vertical)
    private double imageHalfHeight;

    private double xshift;
    private double yshift;

    private final double bottomAngle;
    private final double midAngle;
    private final double halfImageAngle;

    public CameraConfig(double cameraHeight, double bottomDist, double middleDist,
                        double middleWidth, double imageWidth, double imageHeight, double xshift,
                        double yshift) {
        this.cameraHeight = cameraHeight;
        this.bottomDist = bottomDist;
        this.middleDist = middleDist;
        this.middleWidth = middleWidth;
        this.imageHalfWidth = imageWidth * 0.5;
        this.imageHalfHeight = imageHeight * 0.5;
        this.xshift = xshift;
        this.yshift = yshift;

        double heightReciprocal = 1.0 / cameraHeight;
        bottomAngle = Math.atan(bottomDist * heightReciprocal);
        midAngle = Math.atan(middleDist * heightReciprocal);
        halfImageAngle = midAngle - bottomAngle;
    }

    // Converts the pixel position relative to image center to field position relative to camera
    // position. Orientation: up - positive X; left - positive Y
    public Pose2d imagePosToFieldPos(Pose2d pos) {
        double objAngle = halfImageAngle * pos.position.y / imageHalfHeight;
        double x = cameraHeight * Math.tan(midAngle + objAngle);
        double y = -middleWidth * x * pos.position.x / middleDist / imageHalfWidth;
        return new Pose2d(x - xshift, y - yshift, 0.0);
    }
}
