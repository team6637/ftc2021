package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    private int position = 0;
    private Mat mat;

    private final static int THRESHOLD = 100;

    private final static Rect LEFT = new Rect(
            new Point(0, 0),
            new Point(106, 240)
    );

    private final static Rect CENTER = new Rect(
            new Point(107, 0),
            new Point(213, 240)
    );

    private final static Rect RIGHT = new Rect(
            new Point(214, 0),
            new Point(320, 240)
    );

    @Override
    public Mat processFrame(Mat input) {
        mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(50,70,80);
        Scalar highHSV = new Scalar(80,255,255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT);
        Mat center = mat.submat(CENTER);
        Mat right = mat.submat(RIGHT);

        double leftValue = Core.sumElems(left).val[0] / LEFT.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT.area() / 255;

        left.release();
        center.release();
        right.release();

        if(leftValue > centerValue && leftValue > rightValue) {
            position = 1;
        } else if(centerValue > leftValue && centerValue > rightValue) {
            position = 2;
        } else {
            position = 3;
        }


        // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        // Imgproc.rectangle(mat,LEFT,new Scalar(0,255,0));
        // Imgproc.rectangle(mat, CENTER, new Scalar(0, 255, 0));
        // Imgproc.rectangle(mat,RIGHT,new Scalar(0,255,0));

        return mat;
    }

    public int getLastResult() {
        return position;
    }
}
