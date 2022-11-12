package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140,220);
    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 80;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }
}
