package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CapstonePipeline extends OpenCvPipeline {

    //Todo: tune pls :)
    Scalar low = new Scalar(0.0, 141.0, 0.0);
    Scalar high = new Scalar(255.0, 230.0, 95.0);

    Mat mask;
    Mat frame;
    Mat output;

    @Override
    public Mat processFrame(Mat input) {

        frame.release();

        frame = new Mat();
        output = new Mat();
        mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC1);

        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(output, low, high, mask);
        Core.bitwise_and(input, input, frame, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(7, 10), 0.0);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if(hierarchy.size().height > 0 && hierarchy.size().width > 0){
            for(int index = 0; index >= 0; index = (int) hierarchy.get(0, index)[0])
                Imgproc.drawContours(frame, contours, index, new Scalar(88, 0, 0), 2);
        }

        return frame;
    }
}
