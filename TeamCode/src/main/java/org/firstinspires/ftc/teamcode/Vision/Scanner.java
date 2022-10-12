package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

public class Scanner extends OpenCvPipeline {

    Mat mat = new Mat();
    Mat leftMat = new Mat();
    Mat midMat = new Mat();
    Mat rightMat = new Mat();

    Rect leftROI = new Rect(new Point(0,0),new Point(320/3.0,240));
    Rect midROI = new Rect(new Point(320/3.0, 0),new Point(2 * 320/3.0, 240));
    Rect RightROI = new Rect(new Point(2 * 320/3.0, 0),new Point(320, 240));

    private Barcode result = null;
    private Telemetry telemetry;

    public Scanner(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(69.0/2.0,100,100);
        Scalar upperBound = new Scalar(156.0/2.0,255,255);
        Core.inRange(mat, lowerBound, upperBound, mat);

        leftMat = mat.submat(leftROI);
        midMat = mat.submat(midROI);
        rightMat = mat.submat(RightROI);

        double leftValue = Core.sumElems(leftMat).val[0];
        double midValue = Core.sumElems(midMat).val[0];
        double rightValue = Core.sumElems(rightMat).val[0];
        telemetry.addData("left", leftValue);
        telemetry.addData("right", rightValue);
        telemetry.addData("middle", midValue);

        double maxValue = Math.max(leftValue, Math.max(midValue, rightValue));
        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);
        if (maxValue == leftValue){
            result = Barcode.LEFT;
            telemetry.addData("TSE", "left");
            Imgproc.rectangle(input, leftROI, matchColor);
            Imgproc.rectangle(input, midROI, mismatchColor);
            Imgproc.rectangle(input, RightROI, mismatchColor);
        }else if (maxValue == midValue){
            result = Barcode.MIDDLE;
            telemetry.addData("TSE", "middle");
            Imgproc.rectangle(input, midROI, matchColor);
            Imgproc.rectangle(input, leftROI, mismatchColor);
            Imgproc.rectangle(input, RightROI, mismatchColor);
        }else{
            result = Barcode.RIGHT;
            telemetry.addData("TSE", "Right");
            Imgproc.rectangle(input, RightROI, matchColor);
            Imgproc.rectangle(input, leftROI, mismatchColor);
            Imgproc.rectangle(input, midROI, mismatchColor);
        }
        telemetry.addData("Barcode", result.toString().toLowerCase());
        telemetry.update();
        return input;
    }
    public Barcode getResult(){
        while (result == null);
        return result;
    }

}
