package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionBlobPipeline extends OpenCvPipeline {
    Mat mat;
    private int position = 0;

    @Override
    public Mat processFrame(Mat input) {
        mat = new Mat();
        //Imgproc.pyrDown(input, mat);
        //Imgproc.pyrDown(mat, mat);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(50,70,80);
        Scalar highHSV = new Scalar(80,255,255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        //Imgproc.dilate(mat, mat, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        int largest_area = 0;
        int largest_contour_index=-1;

        for( int i = 0; i < contours.size(); i++ ) { // iterate through each contour.
            double a = Imgproc.contourArea(contours.get(i),false);  //  Find the area of contour
            if(a  > 200 && a > largest_area){
                largest_area = (int) a;
                largest_contour_index=i;                //Store the index of largest contour
                //bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
            }
        }

        if(largest_contour_index >= 0) {
            Rect rect = Imgproc.boundingRect(contours.get(largest_contour_index));

            //Imgproc.rectangle(mat, rect.tl(), rect.br(), new Scalar(255,0,0), 2);

            double centerOfBlob = rect.x + (rect.width / 2);
            if (centerOfBlob < 70) {
                position = 1;
            } else if (centerOfBlob < 170) {
                position = 2;
            } else {
                position = 3;
            }
        } else {
            position = 3;
        }
        return mat;
    }

    public int getLastResult() {
        return position;
    }

}
