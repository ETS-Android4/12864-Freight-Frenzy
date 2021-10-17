package org.firstinspires.ftc.teamcode.visionTwo;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Contours {

    private Scalar min, max;
    private Mat frame;
    private Mat mask;
    private Mat output;
    private Mat hierarchy;
    private List<MatOfPoint> contours;

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
                Imgproc.drawContours(frame, contours, index, new Scalar(88, 0, 0), 2);
        }
        return frame;
    }

    public List<MatOfPoint> contourList() {
        return contours;
    }

}
