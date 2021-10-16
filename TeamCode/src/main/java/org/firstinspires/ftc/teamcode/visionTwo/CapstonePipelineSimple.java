package org.firstinspires.ftc.teamcode.visionTwo;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapstonePipelineSimple extends OpenCvPipeline {

    //Todo: tune pls :)
    public Scalar low = new Scalar(31, 152, 95);
    public Scalar high = new Scalar(162, 255, 128);

    Contours contours;

    @Override
    public Mat processFrame(Mat input) {
        contours = new Contours(low, high);
        return contours.getContourFrame(input);
    }
}
