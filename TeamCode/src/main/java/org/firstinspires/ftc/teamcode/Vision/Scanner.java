package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Scanner extends OpenCvPipeline {

    Mat mat = new Mat();

    Rect leftROI = new Rect(new Point(0,0),new Point(320/3.0,240));
    Rect midROI = new Rect(new Point(320/3.0, 0),new Point());
    Rect RightROI = new Rect(new Point(),new Point());

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(69.0/2.0,100,100);
        Scalar upperBound = new Scalar(156.0/2.0,255,255);
        Core.inRange(mat, lowerBound, upperBound, mat);
        return null;
    }
}
