package org.firstinspires.ftc.teamcode.vision;

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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CapstonePipeline extends OpenCvPipeline {

    //Todo: tune pls :)
    private Scalar low = new Scalar(31, 152, 95);
    private Scalar high = new Scalar(162, 255, 128);

    private Mat mask;
    private Mat frame;
    private Mat output;
    private Point centroid;

    @Override
    public Mat processFrame(Mat input) {
        frame = new Mat();
        output = new Mat();
        mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC1);

        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(output, low, high, mask);
        Core.bitwise_and(input, input, frame, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(7, 7), 0.0);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            MatOfPoint biggest = new MatOfPoint();
            for (int index = 0; index >= 0; index = (int) hierarchy.get(0, index)[0]) {
                Imgproc.drawContours(frame, contours, index, new Scalar(88, 0, 0), 2);
                if (index > 0 && (contours.get(index).size().area() > contours.get(index - 1).size().area())) {
                    biggest = contours.get(index);
                }
            }

            Moments moments = Imgproc.moments(biggest);
            centroid = new Point();

            centroid.x = moments.get_m10() / moments.get_m00();
            centroid.y = moments.get_m01() / moments.get_m00();

            Rect rect = new Rect((int) centroid.x, (int) centroid.y, 10, 10);
            Imgproc.rectangle(frame, rect, new Scalar(0, 255, 255));

        }

        return frame;
    }

    public Point getCentroid() {
        return centroid;
    }

    public void setLowerBound(Scalar low) {
        this.low = low;
    }

    public void setUpperBound(Scalar high) {
        this.high = high;
    }

}
