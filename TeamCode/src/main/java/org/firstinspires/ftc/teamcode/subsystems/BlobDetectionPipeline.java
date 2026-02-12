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

public class BlobDetectionPipeline extends OpenCvPipeline {
    private String detectedColor = "none";
    private Rect boundingBox = null;

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Masks for purple and green
        Mat maskPurple = new Mat();
        Mat maskGreen = new Mat();

        Core.inRange(hsv, new Scalar(135, 80, 80), new Scalar(165, 255, 255), maskPurple);
        Core.inRange(hsv, new Scalar(40, 70, 70), new Scalar(85, 255, 255), maskGreen);

        // Find blobs
        Rect purpleBlob = findLargestBlob(maskPurple);
        Rect greenBlob = findLargestBlob(maskGreen);

        if (purpleBlob != null && (greenBlob == null || purpleBlob.area() > greenBlob.area())) {
            detectedColor = "purple";
            boundingBox = purpleBlob;
            Imgproc.rectangle(input, purpleBlob, new Scalar(255, 0, 255), 2);
        } else if (greenBlob != null) {
            detectedColor = "green";
            boundingBox = greenBlob;
            Imgproc.rectangle(input, greenBlob, new Scalar(0, 255, 0), 2);
        } else {
            detectedColor = "none";
            boundingBox = null;
        }

        return input;
    }

    private Rect findLargestBlob(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 500; // ignore tiny noise
        Rect bestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }
        return bestRect;
    }

    public String getDetectedColor() {
        return detectedColor;
    }

    public Rect getBoundingBox() {
        return boundingBox;
    }
}
