package org.firstinspires.ftc.teamcode.hardware;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140,230);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 60;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Mat sleeveCb;
    Mat YCrCb = new Mat();
    Mat CbChannel = new Mat();
    private volatile int meanCbValue;

    void inputToYCrCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, CbChannel, 2);
    }
    @Override
    public void init(Mat firstFrame)
    {
        inputToYCrCb(firstFrame);
        sleeveCb = CbChannel.submat(new Rect(region1_pointA, region1_pointB));
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        inputToYCrCb(input);
        meanCbValue = (int) Core.mean(sleeveCb).val[0];
        return input;
    }
    public int getMeanCbValue()
    {
        return meanCbValue;
    }
}