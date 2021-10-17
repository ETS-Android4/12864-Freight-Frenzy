package org.firstinspires.ftc.teamcode.visionTwo;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class Contours {

    private Scalar min, max;
    private Mat frame;
    private Mat mask;
    private Mat output;
    private Mat hierarchy;
    private List<MatOfPoint> contours;

    private Scalar contourColor = new Scalar(0, 55, 0);
    private int contourThickness = 2;

    public Contours(Scalar min, Scalar max) {
        this.min = min;
        this.max = max;

        frame = new Mat();
        output = new Mat();
        mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC1);
    }

    public Contours(Scalar min, Scalar max, Mat frame, Mat output, Mat mask) {
        this.min = min;
        this.max = max;

        this.frame = frame;
        this.output = output;
        this.mask = mask;
    }

    public Mat getContourFrame(Mat input) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(output, min, max, mask);
        Core.bitwise_and(input, input, frame, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(7, 7), 0.0);
        contours = new ArrayList<>();
        hierarchy = new Mat();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            for (int index = 0; index >= 0; index = (int) hierarchy.get(0, index)[0])
                Imgproc.drawContours(frame, contours, index, contourColor, contourThickness);
        }
        return frame;
    }

    public List<MatOfPoint> getContourList() {
        return contours;
    }

    public Point drawCentroid(Mat array) {
        Moments moments = Imgproc.moments(array);
        Point centroid = new Point();

        centroid.x = moments.get_m10() / moments.get_m00();
        centroid.y = moments.get_m01() / moments.get_m00();

        return centroid;
    }

    public void setLowerBound(Scalar low) {
        min = low;
    }

    public void setUpperBound(Scalar high) {
        max = high;
    }

    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        min = low;
        max = high;
    }

    public void setContourColor(Scalar color) {
        contourColor = color;
    }

    public void setContourThickness(int thickness) {
        contourThickness = thickness;
    }

    public Mat getMask() {
        return mask;
    }

    public Mat getOutput() {
        return output;
    }

}
